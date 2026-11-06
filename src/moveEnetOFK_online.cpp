// Online upper-body MoveEnet + SAE optical flow for PRIMI.
// Consumes the locally republished event stream; never opens robot hardware ports.
#include <event-driven/core.h>
#include <event-driven/algs.h>
#include <hpe-core/utility.h>
#include <hpe-core/representations.h>
#include <hpe-core/motion.h>
#include <hpe-core/fusion.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <functional>
#include <stdexcept>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
using Mono = yarp::sig::ImageOf<yarp::sig::PixelMono>;
using Rgb = yarp::sig::ImageOf<yarp::sig::PixelRgb>;

std::atomic<bool> stopRequested{false};
void signalHandler(int) { stopRequested.store(true); }

constexpr std::array<int, 7> upperJoints = {
    hpecore::head, hpecore::shoulderR, hpecore::shoulderL,
    hpecore::elbowR, hpecore::elbowL, hpecore::handR, hpecore::handL
};

struct Config {
    std::string camera{"left"};
    int width{640};
    int height{480};
    double fNet{20.0};
    double fFlow{200.0};
    double fVis{30.0};
    double confidenceThreshold{0.2};
    double trackTimeout{1.0};
    double procU{0.77};
    double measUD{0.06};
    double measUV{0.97};  // Retained for offline configuration parity; upstream LC ignores it.
    bool useLC{true};
    int roi{20};
    // Event-domain denoising, upstream of both EROS and SAE.
    bool noiseFilter{true};
    double noiseRefractoryMs{1.0};
    double noiseSpatialMs{5.0};
    int noiseRadius{1};
    // Visualization-only toggles: the estimators keep running in both cases.
    bool showMovenet{true};
    bool showOfk{true};
};

bool parseConfig(yarp::os::ResourceFinder &rf, Config &cfg) {
    using yarp::os::Value;
    cfg.camera = rf.check("camera", Value("left")).asString();
    cfg.width = rf.check("width", Value(640)).asInt32();
    cfg.height = rf.check("height", Value(480)).asInt32();
    cfg.fNet = rf.check("f_net", Value(20.0)).asFloat64();
    cfg.fFlow = rf.check("f_flow", Value(200.0)).asFloat64();
    cfg.fVis = rf.check("f_vis", Value(30.0)).asFloat64();
    cfg.confidenceThreshold = rf.check("confidence_th", Value(0.2)).asFloat64();
    cfg.trackTimeout = rf.check("track_timeout", Value(1.0)).asFloat64();
    cfg.procU = rf.check("pu", Value(0.77)).asFloat64();
    cfg.measUD = rf.check("muD", Value(0.06)).asFloat64();
    cfg.measUV = rf.check("muV", Value(0.97)).asFloat64();
    cfg.roi = rf.check("roi", Value(20)).asInt32();
    cfg.useLC = rf.check("use_lc", Value(true)).asBool();
    cfg.noiseFilter = rf.check("noise_filter", Value(true)).asBool();
    cfg.noiseRefractoryMs = rf.check("noise_refractory_ms", Value(1.0)).asFloat64();
    cfg.noiseSpatialMs = rf.check("noise_spatial_ms", Value(5.0)).asFloat64();
    cfg.noiseRadius = rf.check("noise_radius", Value(1)).asInt32();
    cfg.showMovenet = rf.check("show_movenet", Value(true)).asBool();
    cfg.showOfk = rf.check("show_ofk", Value(true)).asBool();
    if ((cfg.camera != "left" && cfg.camera != "right") ||
        cfg.width < 16 || cfg.height < 16 ||
        cfg.fNet <= 0 || cfg.fFlow <= 0 || cfg.fVis <= 0 ||
        cfg.roi < 3 || cfg.roi >= cfg.width || cfg.roi >= cfg.height ||
        cfg.confidenceThreshold < 0 || cfg.confidenceThreshold > 1 ||
        cfg.trackTimeout <= 0 || cfg.trackTimeout > 4.0 ||
        cfg.procU <= 0 || cfg.measUD <= 0 ||
        !std::isfinite(cfg.noiseRefractoryMs) ||
        !std::isfinite(cfg.noiseSpatialMs) ||
        cfg.noiseRefractoryMs < 0 || cfg.noiseSpatialMs < 0 ||
        cfg.noiseRadius < 1 || cfg.noiseRadius > 3) {
        std::cerr << "Invalid arguments. Camera is left/right; rates and noise > 0; "
                     "track_timeout in (0,4]; width/height and ROI must be valid; "
                     "noise windows must be nonnegative and radius in [1,3].\n";
        return false;
    }
    return true;
}

bool isUpperJoint(int i) {
    for (int id : upperJoints) if (id == i) return true;
    return false;
}

bool validCoord(double x, double y, const Config &cfg) {
    return std::isfinite(x) && std::isfinite(y) &&
           x >= 0 && y >= 0 && x < cfg.width && y < cfg.height;
}

// Causal background-activity filter. A spatially supported event must have
// at least one DIFFERENT pixel active recently (the current pixel alone is
// not evidence of motion). We record all raw observations, including rejected
// ones, so neighboring activity can still provide support. Only one thread
// accesses this object (acquisitionLoop): no additional locking required.
// Time units are seconds, on the same relative axis as the SAE.
class EventNoiseFilter {
public:
    enum class Decision { Keep, Refractory, NoSpatialSupport };

    explicit EventNoiseFilter(const Config &cfg)
        : width_(cfg.width), height_(cfg.height),
          enabled_(cfg.noiseFilter),
          refractory_(cfg.noiseRefractoryMs * 0.001),
          spatialWindow_(cfg.noiseSpatialMs * 0.001),
          radius_(cfg.noiseRadius),
          lastSeen_(static_cast<std::size_t>(cfg.width) * cfg.height, -1.0),
          lastPolarity_(static_cast<std::size_t>(cfg.width) * cfg.height, 2) {}

    Decision check(int x, int y, int polarity, double t) {
        if (!enabled_) return Decision::Keep;
        const std::size_t idx = static_cast<std::size_t>(y) * width_ + x;
        const double previous = lastSeen_[idx];
        const bool refractory = refractory_ > 0 && previous >= 0 &&
            lastPolarity_[idx] == static_cast<std::uint8_t>(polarity) &&
            t >= previous && t - previous < refractory_;

        bool spatiallySupported = spatialWindow_ <= 0;
        if (!refractory && !spatiallySupported) {
            const int x0 = std::max(0, x - radius_);
            const int x1 = std::min(width_ - 1, x + radius_);
            const int y0 = std::max(0, y - radius_);
            const int y1 = std::min(height_ - 1, y + radius_);
            for (int ny = y0; ny <= y1 && !spatiallySupported; ++ny) {
                for (int nx = x0; nx <= x1; ++nx) {
                    if (nx == x && ny == y) continue;
                    const double nt = lastSeen_[static_cast<std::size_t>(ny) * width_ + nx];
                    if (nt >= 0 && t >= nt && t - nt <= spatialWindow_) {
                        spatiallySupported = true;
                        break;
                    }
                }
            }
        }

        // Update even for rejected events to preserve neighboring evidence.
        // For out-of-order individual timestamps, never move the recorded
        // timestamp backwards (the source packet clock is checked separately).
        if (t >= previous) {
            lastSeen_[idx] = t;
            lastPolarity_[idx] = static_cast<std::uint8_t>(polarity);
        }
        if (refractory) return Decision::Refractory;
        if (!spatiallySupported) return Decision::NoSpatialSupport;
        return Decision::Keep;
    }

private:
    int width_, height_;
    bool enabled_;
    double refractory_, spatialWindow_;
    int radius_;
    std::vector<double> lastSeen_;
    std::vector<std::uint8_t> lastPolarity_;
};

struct Request {
    int sequence{0};
    double frameTimestamp{0};    // Relative tracking time (precision-safe).
    double sourceTimestamp{0};   // Original YARP envelope time.
    Clock::time_point started{};
    bool warned{false};
};

struct Shared {
    explicit Shared(const Config &c) : cfg(c), noiseFilter(c) {
        eros.init(cfg.width, cfg.height, 7, 0.3);
        sae.init(cfg.width, cfg.height);
        // Value-initialization is essential: the upstream LC implementation
        // contains fixed-size velocity/history arrays without member initializers.
        if (!tracker.initialise({cfg.procU, cfg.measUD, cfg.measUV,
                                cfg.useLC ? 1.0 : 0.0})) {
            throw std::runtime_error("LC state initialization failed");
        }
        lastDetection.fill(-1.0);
        confidence.fill(0.0);
        flowEstimator.prev_update_ts = 0;
    }
    Config cfg;
    EventNoiseFilter noiseFilter;
    std::atomic<bool> running{true};
    std::mutex surfacesMutex;
    ev::EROS eros;
    hpecore::SAE sae;
    double eventTimestamp{0.0};        // Relative time used by SAE and LC.
    double eventSourceTimestamp{0.0};  // Original source time for YARP envelopes.
    double sourceEpoch{0.0};
    bool haveEvents{false};

    std::mutex trackerMutex;
    hpecore::multiJointLatComp tracker{};
    hpecore::pwtripletvelocity flowEstimator{};
    std::array<double, 13> lastDetection{};
    std::array<double, 13> confidence{};
    hpecore::skeleton13_vel lastVelocity{};

    // The last unfiltered-by-Kalman pose returned by MoveEnet.
    // It is NOT updated by the OF/KF estimator. Protected by trackerMutex.
    hpecore::skeleton13 rawMovenetPose{};
    std::array<double, 13> rawMovenetConfidence{};
    double rawMovenetTimestamp{-1.0};
    bool haveRawMovenet{false};

    std::mutex requestMutex;
    std::optional<Request> pending;
    int nextRequestSequence{0};

    std::atomic<std::uint64_t> packets{0}, events{0}, rawEvents{0},
        invalidEvents{0}, refractoryDrops{0}, spatialDrops{0},
        requests{0}, modelRepliesReceived{0}, detections{0}, flowSteps{0};
};

// Copies only after taking the surface lock; cv::Mat shallow copies would race.
bool snapshotEROS(Shared &s, cv::Mat &dst, double &timestamp,
                  double &sourceTimestamp) {
    std::lock_guard<std::mutex> lk(s.surfacesMutex);
    if (!s.haveEvents) return false;
    dst = s.eros.getSurface().clone();
    timestamp = s.eventTimestamp;
    sourceTimestamp = s.eventSourceTimestamp;
    return true;
}

bool snapshotSAE(Shared &s, cv::Mat &dst, double &timestamp,
                 double &sourceTimestamp) {
    std::lock_guard<std::mutex> lk(s.surfacesMutex);
    if (!s.haveEvents) return false;
    dst = s.sae.getSurface().clone();
    timestamp = s.eventTimestamp;
    sourceTimestamp = s.eventSourceTimestamp;
    return true;
}

void acquisitionLoop(Shared &s, ev::BufferedPort<ev::AE> &input) {
    bool warnedNoStamp = false;
    double previousTimestamp = 0.0;
    while (s.running.load() && !stopRequested.load()) {
        ev::packet<ev::AE> *packet = input.read(true);
        if (!packet) break;
        if (packet->size() == 0) continue;

        double t = packet->timestamp();
        if (!std::isfinite(t) || t <= 0) {
            t = yarp::os::Time::now();
            if (!warnedNoStamp) {
                std::cerr << "[WARN] Source packet has no usable YARP envelope; "
                             "using local receipt time. Check synchronization.\n";
                warnedNoStamp = true;
            }
        }
        if (previousTimestamp > 0 && t + 0.001 < previousTimestamp) {
            std::cerr << "[ERROR] Event timestamps moved backwards; stop/restart "
                         "to avoid incorrect latency compensation.\n";
            s.running.store(false);
            stopRequested.store(true);
            break;
        }
        previousTimestamp = t;

        const unsigned int lastTick = (*packet)[packet->size() - 1].ts;
        std::uint64_t validInPacket = 0;
        std::uint64_t rawInPacket = 0;
        std::uint64_t invalidInPacket = 0;
        std::uint64_t refractoryInPacket = 0;
        std::uint64_t spatialInPacket = 0;
        std::lock_guard<std::mutex> lk(s.surfacesMutex);
        // Use a relative clock in SAE: absolute Unix timestamps lose precision
        // for the microsecond differences required by triplet optical flow.
        if (!s.haveEvents) s.sourceEpoch = t;
        const double relativeTime = 1.0 + (t - s.sourceEpoch);
        for (auto &v : *packet) {
            const int x = static_cast<int>(v.x);
            const int y = static_cast<int>(v.y);
            if (x < 0 || y < 0 || x >= s.cfg.width || y >= s.cfg.height) {
                ++invalidInPacket;
                continue;
            }
            double eventTime = relativeTime;
            // If event-driven was built with individual timestamps, use them.
            // Otherwise packet-time SAE is a fallback with reduced flow accuracy.
            if (ev::ts_status) {
                const double offset = ev::deltaS(
                    static_cast<int>(lastTick), static_cast<int>(v.ts));
                if (offset >= 0 && offset < 2.0) eventTime = relativeTime - offset;
            }
            ++rawInPacket;
            const auto decision = s.noiseFilter.check(x, y, v.p, eventTime);
            if (decision == EventNoiseFilter::Decision::Refractory) {
                ++refractoryInPacket;
                continue;
            }
            if (decision == EventNoiseFilter::Decision::NoSpatialSupport) {
                ++spatialInPacket;
                continue;
            }
            s.eros.update(x, y);
            s.sae.update(x, y, eventTime);
            ++validInPacket;
        }
        s.events.fetch_add(validInPacket);
        s.rawEvents.fetch_add(rawInPacket);
        s.invalidEvents.fetch_add(invalidInPacket);
        s.refractoryDrops.fetch_add(refractoryInPacket);
        s.spatialDrops.fetch_add(spatialInPacket);
        s.eventTimestamp = relativeTime;
        s.eventSourceTimestamp = t;
        s.haveEvents = true;
        ++s.packets;
    }
    s.running.store(false);
}

// The acquisition thread never waits for inference. Only one request is in
// flight; if the model is slower than f_net, frames are SKIPPED, not queued.
void requestLoop(Shared &s, yarp::os::BufferedPort<Mono> &output) {
    const auto period = std::chrono::duration<double>(1.0 / s.cfg.fNet);
    auto next = Clock::now();
    while (s.running.load() && !stopRequested.load()) {
        next += std::chrono::duration_cast<Clock::duration>(period);
        if (output.getOutputCount() > 0) {
            bool canSend = false;
            {
                std::lock_guard<std::mutex> lk(s.requestMutex);
                if (!s.pending) canSend = true;
                else if (!s.pending->warned &&
                         std::chrono::duration<double>(Clock::now() -
                             s.pending->started).count() > 2.0) {
                    std::cerr << "[WARN] Inference response exceeded 2 seconds; "
                                 "no additional frames queued. Check Python logs.\n";
                    s.pending->warned = true;
                }
            }
            if (canSend) {
                cv::Mat eros;
                double ts = 0, sourceTs = 0;
                if (snapshotEROS(s, eros, ts, sourceTs)) {
                    cv::Mat mono;
                    eros.convertTo(mono, CV_8U);
                    cv::GaussianBlur(mono, mono, {5, 5}, 0);
                    int seq;
                    {
                        std::lock_guard<std::mutex> lk(s.requestMutex);
                        seq = ++s.nextRequestSequence;
                        s.pending = Request{seq, ts, sourceTs, Clock::now(), false};
                    }
                    Mono &img = output.prepare();
                    img.resize(mono.cols, mono.rows);
                    cv::Mat view(img.height(), img.width(), CV_8UC1,
                                 img.getRawImage(), img.getRowSize());
                    mono.copyTo(view);
                    yarp::os::Stamp stamp(seq, sourceTs);
                    output.setEnvelope(stamp);
                    output.write();
                    ++s.requests;
                }
            }
        }
        std::this_thread::sleep_until(next);
        if (next < Clock::now() - std::chrono::seconds(1)) next = Clock::now();
    }
}

bool decodeSkeleton(const yarp::os::Bottle &msg, const Config &cfg,
                    hpecore::skeleton13 &joints,
                    std::array<double, 13> &confidence) {
    if (msg.size() != 2 || msg.get(0).asString() != "SKLT") return false;
    const yarp::os::Bottle *a = msg.get(1).asList();
    if (!a || a->size() != 39) return false;
    for (int i = 0; i < 13; ++i) {
        const double x = a->get(2 * i).asFloat64();
        const double y = a->get(2 * i + 1).asFloat64();
        double conf = a->get(26 + i).asFloat64();
        if (!std::isfinite(conf) || !isUpperJoint(i) ||
            conf < cfg.confidenceThreshold || !validCoord(x, y, cfg)) {
            joints[i] = {0, 0};
            confidence[i] = 0;
        } else {
            joints[i] = {static_cast<float>(x), static_cast<float>(y)};
            confidence[i] = conf;
        }
    }
    return true;
}

// Published at the source estimator's rate (not the GUI's f_vis).
void publishPose(yarp::os::BufferedPort<yarp::os::Bottle> &port,
                 const char *type, const hpecore::skeleton13 &pose,
                 const std::array<double, 13> &confidence, int id, double ts);

void responseLoop(Shared &s, yarp::os::BufferedPort<yarp::os::Bottle> &input,
                  yarp::os::BufferedPort<yarp::os::Bottle> &rawPosePort) {
    while (s.running.load() && !stopRequested.load()) {
        const yarp::os::Bottle *msg = input.read(true);
        if (!msg) break;
        yarp::os::Stamp stamp;
        if (!input.getEnvelope(stamp)) {
            std::cerr << "[WARN] Inference result without source timestamp ignored.\n";
            continue;
        }
        Request matched;
        {
            std::lock_guard<std::mutex> lk(s.requestMutex);
            if (!s.pending || stamp.getCount() != s.pending->sequence ||
                std::abs(stamp.getTime() - s.pending->sourceTimestamp) > 1e-4) {
                std::cerr << "[WARN] Unmatched inference result ignored (request id "
                          << stamp.getCount() << ").\n";
                continue;
            }
            matched = *s.pending;
            s.pending.reset();
        }

        hpecore::skeleton13 detected{};
        std::array<double, 13> confidences{};
        if (!decodeSkeleton(*msg, s.cfg, detected, confidences)) {
            std::cerr << "[WARN] Invalid model output Bottle, expected SKLT + 39 floats.\n";
            continue;
        }
        bool anyValid = false;
        for (double c : confidences) anyValid = anyValid || c > 0;

        {
            std::lock_guard<std::mutex> lk(s.trackerMutex);
            // Store EXACT MoveEnet detections, before any state-estimator update.
            // Even an all-invalid result replaces the previous detection, so
            // the red skeleton cannot remain stuck on a past valid pose.
            s.rawMovenetPose = detected;
            s.rawMovenetConfidence = confidences;
            s.rawMovenetTimestamp = matched.frameTimestamp;
            s.haveRawMovenet = true;

            if (anyValid) {
                if (!s.tracker.poseIsInitialised()) {
                    s.tracker.set(detected, matched.frameTimestamp);
                    for (int j = 0; j < 13; ++j) {
                        if (confidences[j] > 0) {
                            s.lastDetection[j] = matched.frameTimestamp;
                            s.confidence[j] = confidences[j];
                        }
                    }
                } else {
                    for (int j : upperJoints) {
                        if (confidences[j] == 0) continue;
                        if (matched.frameTimestamp <= s.lastDetection[j]) continue;
                        s.tracker.updateFromPosition(
                            static_cast<hpecore::jointName>(j), detected[j],
                            matched.frameTimestamp);
                        s.lastDetection[j] = matched.frameTimestamp;
                        s.confidence[j] = confidences[j];
                    }
                }
                ++s.detections;
            }
        }
        ++s.modelRepliesReceived;
        // This port updates only when a MoveEnet reply is received (<= f_net).
        // The envelope uses the ORIGINAL captured EROS frame timestamp.
        publishPose(rawPosePort, "SKLT", detected, confidences,
                    matched.sequence, matched.sourceTimestamp);
    }
}

void flowLoop(Shared &s,
              yarp::os::BufferedPort<yarp::os::Bottle> &flowPosePort) {
    const auto period = std::chrono::duration<double>(1.0 / s.cfg.fFlow);
    auto next = Clock::now();
    double previousFlowTime = 0.0;
    int poseSeq = 0;
    while (s.running.load() && !stopRequested.load()) {
        next += std::chrono::duration_cast<Clock::duration>(period);
        cv::Mat sae;
        double ts = 0, sourceTs = 0;
        if (snapshotSAE(s, sae, ts, sourceTs) && ts > previousFlowTime) {
            previousFlowTime = ts;
            hpecore::skeleton13 updatedPose{};
            std::array<double, 13> updatedConfidence{};
            bool haveUpdatedPose = false;
            {
                // A single thread owns the upstream velocity estimator. Its
                // internal prev_update_ts is not thread safe.
                std::lock_guard<std::mutex> lk(s.trackerMutex);
                if (!s.tracker.poseIsInitialised()) {
                    s.flowEstimator.prev_update_ts = ts;
                } else {
                    auto pose = s.tracker.query();
                    for (int j = 0; j < 13; ++j) {
                        if (!isUpperJoint(j) || s.lastDetection[j] < 0 ||
                            ts - s.lastDetection[j] > s.cfg.trackTimeout ||
                            ts <= s.lastDetection[j]) {
                            pose[j] = {0, 0};
                        }
                    }
                    auto flow = s.flowEstimator.multi_area_velocity(
                        sae, ts, pose, s.cfg.roi);
                    // The skeleton13 overload in hpe-core scales flow by 1.4.
                    // Cancel this factor to match MoveEnetOFK offline.cpp.
                    for (int j = 0; j < 13; ++j) {
                        if (pose[j].u == 0 && pose[j].v == 0) {
                            flow[j] = {0, 0};
                        } else {
                            flow[j].u /= 1.4f;
                            flow[j].v /= 1.4f;
                        }
                    }
                    s.tracker.setVelocity(flow);
                    for (int j = 0; j < 13; ++j) {
                        if (pose[j].u == 0 && pose[j].v == 0) continue;
                        s.tracker.updateFromVelocity(
                            static_cast<hpecore::jointName>(j), flow[j], ts);
                    }
                    s.lastVelocity = flow;
                    ++s.flowSteps;
                    updatedPose = s.tracker.query();
                    for (int j = 0; j < 13; ++j) {
                        if (!isUpperJoint(j) || s.lastDetection[j] < 0 ||
                            ts - s.lastDetection[j] > s.cfg.trackTimeout) {
                            updatedPose[j] = {0, 0};
                        } else {
                            updatedConfidence[j] = s.confidence[j];
                        }
                    }
                    haveUpdatedPose = true;
                }
            }
            // Publish once for each actual OF/KF update, not at f_vis. Do not
            // hold trackerMutex while writing to a potentially slow subscriber.
            if (haveUpdatedPose) {
                publishPose(flowPosePort, "SKLT", updatedPose,
                            updatedConfidence, ++poseSeq, sourceTs);
            }
        }
        std::this_thread::sleep_until(next);
        if (next < Clock::now() - std::chrono::seconds(1)) next = Clock::now();
    }
}

void publishMono(yarp::os::BufferedPort<Mono> &port, const cv::Mat &mono,
                 int id, double ts) {
    if (port.getOutputCount() == 0) return;
    Mono &img = port.prepare();
    img.resize(mono.cols, mono.rows);
    cv::Mat view(img.height(), img.width(), CV_8UC1,
                 img.getRawImage(), img.getRowSize());
    mono.copyTo(view);
    yarp::os::Stamp stamp(id, ts);
    port.setEnvelope(stamp);
    port.write();
}

void publishOverlay(yarp::os::BufferedPort<Rgb> &port, const cv::Mat &rgb,
                    int id, double ts) {
    if (port.getOutputCount() == 0) return;
    Rgb &img = port.prepare();
    img.resize(rgb.cols, rgb.rows);
    cv::Mat view(img.height(), img.width(), CV_8UC3,
                 img.getRawImage(), img.getRowSize());
    rgb.copyTo(view);
    yarp::os::Stamp stamp(id, ts);
    port.setEnvelope(stamp);
    port.write();
}

void publishPose(yarp::os::BufferedPort<yarp::os::Bottle> &port,
                 const char *type, const hpecore::skeleton13 &pose,
                 const std::array<double, 13> &confidence, int id, double ts) {
    if (port.getOutputCount() == 0) return;
    auto &msg = port.prepare();
    msg.clear();
    msg.addString(type);
    auto &arr = msg.addList();
    for (const auto &j : pose) {
        arr.addFloat64(j.u);
        arr.addFloat64(j.v);
    }
    for (double c : confidence) arr.addFloat64(c);
    yarp::os::Stamp stamp(id, ts);
    port.setEnvelope(stamp);
    port.write();
}

// Draw the 7 upper-body joints only. The circles are MoveEnet measurements,
// crosses are the OF+KF tracker estimate. Pixel coordinates and confidence
// have already been validated by decodeSkeleton()/the pose mask.
void drawUpperPose(cv::Mat &bgr, const hpecore::skeleton13 &pose,
                   const std::array<double, 13> &confidence,
                   const Config &cfg, const cv::Scalar &color,
                   int lineThickness, bool drawCircles) {
    const auto visible = [&](int id) {
        return confidence[id] > 0 && validCoord(pose[id].u, pose[id].v, cfg);
    };
    const auto point = [&](int id) {
        return cv::Point(static_cast<int>(std::lround(pose[id].u)),
                         static_cast<int>(std::lround(pose[id].v)));
    };
    constexpr std::array<std::array<int, 2>, 5> links{{
        {{hpecore::shoulderR, hpecore::shoulderL}},
        {{hpecore::shoulderR, hpecore::elbowR}},
        {{hpecore::elbowR, hpecore::handR}},
        {{hpecore::shoulderL, hpecore::elbowL}},
        {{hpecore::elbowL, hpecore::handL}}
    }};
    for (const auto &link : links) {
        if (visible(link[0]) && visible(link[1])) {
            cv::line(bgr, point(link[0]), point(link[1]), color,
                     lineThickness, cv::LINE_AA);
        }
    }
    if (visible(hpecore::head) && visible(hpecore::shoulderR) &&
        visible(hpecore::shoulderL)) {
        const cv::Point right = point(hpecore::shoulderR);
        const cv::Point left = point(hpecore::shoulderL);
        const cv::Point neck((right.x + left.x) / 2,
                             (right.y + left.y) / 2);
        cv::line(bgr, point(hpecore::head), neck, color,
                 lineThickness, cv::LINE_AA);
    }
    for (int id : upperJoints) {
        if (!visible(id)) continue;
        if (drawCircles) {
            cv::circle(bgr, point(id), 5, color, 2, cv::LINE_AA);
        } else {
            cv::drawMarker(bgr, point(id), color, cv::MARKER_CROSS,
                           10, 2, cv::LINE_AA);
        }
    }
}

void visualizationLoop(Shared &s,
                       yarp::os::BufferedPort<Mono> &erosPort,
                       yarp::os::BufferedPort<Rgb> &overlayPort,
                       yarp::os::BufferedPort<yarp::os::Bottle> &skeletonPort,
                       yarp::os::BufferedPort<yarp::os::Bottle> &velocityPort) {
    const auto period = std::chrono::duration<double>(1.0 / s.cfg.fVis);
    auto next = Clock::now();
    int seq = 0;
    auto lastRateCheck = Clock::now();
    std::uint64_t previousNetReplies = 0, previousFlowSteps = 0;
    double achievedNetHz = 0, achievedFlowHz = 0;
    while (s.running.load() && !stopRequested.load()) {
        next += std::chrono::duration_cast<Clock::duration>(period);
        const auto now = Clock::now();
        const double rateWindow =
            std::chrono::duration<double>(now - lastRateCheck).count();
        if (rateWindow >= 1.0) {
            const auto net = s.modelRepliesReceived.load();
            const auto flow = s.flowSteps.load();
            achievedNetHz = (net - previousNetReplies) / rateWindow;
            achievedFlowHz = (flow - previousFlowSteps) / rateWindow;
            previousNetReplies = net;
            previousFlowSteps = flow;
            lastRateCheck = now;
        }
        cv::Mat eros;
        double ts = 0, sourceTs = 0;
        if (snapshotEROS(s, eros, ts, sourceTs)) {
            const int id = ++seq;
            hpecore::skeleton13 pose{};
            hpecore::skeleton13_vel velocity{};
            std::array<double, 13> conf{};
            hpecore::skeleton13 rawPose{};
            std::array<double, 13> rawConf{};
            bool rawPoseRecent = false;
            double rawAgeMs = 0.0;
            {
                std::lock_guard<std::mutex> lk(s.trackerMutex);
                if (s.haveRawMovenet && s.rawMovenetTimestamp <= ts &&
                    ts - s.rawMovenetTimestamp <= s.cfg.trackTimeout) {
                    rawPose = s.rawMovenetPose;
                    rawConf = s.rawMovenetConfidence;
                    rawAgeMs = (ts - s.rawMovenetTimestamp) * 1000.0;
                    rawPoseRecent = true;
                }
                if (s.tracker.poseIsInitialised()) {
                    pose = s.tracker.query();
                    velocity = s.lastVelocity;
                    for (int j = 0; j < 13; ++j) {
                        if (!isUpperJoint(j) || s.lastDetection[j] < 0 ||
                            ts - s.lastDetection[j] > s.cfg.trackTimeout) {
                            pose[j] = {0, 0};
                            velocity[j] = {0, 0};
                        } else {
                            conf[j] = s.confidence[j];
                        }
                    }
                }
            }
            cv::Mat mono;
            eros.convertTo(mono, CV_8U);
            publishMono(erosPort, mono, id, sourceTs);
            publishPose(skeletonPort, "SKLT", pose, conf, id, sourceTs);
            publishPose(velocityPort, "JVEL", velocity, conf, id, sourceTs);
            if (overlayPort.getOutputCount() > 0) {
                cv::Mat enhanced;
                cv::GaussianBlur(mono, enhanced, {3, 3}, 0);
                // cv::normalize(enhanced, enhanced, 0, 255, cv::NORM_MINMAX);
                // Draw in OpenCV BGR, then convert to YARP PixelRgb. Without
                // this conversion red would appear BLUE in yarpview.
                cv::Mat bgr;
                cv::cvtColor(enhanced, bgr, cv::COLOR_GRAY2BGR);
                // First draw thick green OF+KF, then thinner raw red NN lines.
                // Circles vs crosses keep the joints distinguishable even when
                // the coordinates overlap completely.
                if (s.cfg.showOfk) {
                    drawUpperPose(bgr, pose, conf, s.cfg,
                                  cv::Scalar(0, 255, 0), 2, false);
                }
                if (s.cfg.showMovenet && rawPoseRecent) {
                    drawUpperPose(bgr, rawPose, rawConf, s.cfg,
                                  cv::Scalar(0, 0, 255), 1, true);
                }
                cv::rectangle(bgr, cv::Point(4, 4),
                              cv::Point(std::min(410, bgr.cols - 1),
                                        std::min(52, bgr.rows - 1)),
                              cv::Scalar(0, 0, 0), cv::FILLED);
                const std::string redLabel =
                    "MoveEnet RED | " +
                    std::to_string(static_cast<int>(std::lround(achievedNetHz))) +
                    "/" +
                    std::to_string(static_cast<int>(std::lround(s.cfg.fNet))) +
                    " Hz | " +
                    (rawPoseRecent
                         ? std::to_string(static_cast<int>(std::lround(rawAgeMs))) + " ms old"
                         : std::string("no recent detection"));
                const std::string greenLabel =
                    "OF+KF GREEN | " +
                    std::to_string(static_cast<int>(std::lround(achievedFlowHz))) +
                    "/" +
                    std::to_string(static_cast<int>(std::lround(s.cfg.fFlow))) +
                    " Hz";
                if (s.cfg.showMovenet) {
                    cv::putText(bgr, redLabel, {10, 20},
                                cv::FONT_HERSHEY_SIMPLEX, 0.45,
                                cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
                }
                if (s.cfg.showOfk) {
                    cv::putText(bgr, greenLabel, {10, 42},
                                cv::FONT_HERSHEY_SIMPLEX, 0.45,
                                cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                }
                cv::Mat rgb;
                cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
                publishOverlay(overlayPort, rgb, id, sourceTs);
            }
        }
        std::this_thread::sleep_until(next);
        if (next < Clock::now() - std::chrono::seconds(1)) next = Clock::now();
    }
}

}  // namespace

int main(int argc, char *argv[]) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    if (rf.check("help")) {
        std::cout << "moveEnetOFK_online --camera left|right --width 640 "
                     "--height 480 --f_net 20 --f_flow 200 --f_vis 30 "
                     "--confidence_th 0.2 --roi 20 --pu 0.77 --muD 0.06 "
                     "--use_lc true --noise_filter true "
                     "--noise_refractory_ms 1.0 --noise_spatial_ms 5.0 "
                     "--noise_radius 1 --show_movenet true --show_ofk true\n";
        return 0;
    }
    Config cfg;
    if (!parseConfig(rf, cfg)) return 1;
    yarp::os::Network network;
    if (!yarp::os::Network::checkNetwork(2.0)) {
        std::cerr << "YARP name server unavailable.\n";
        return 1;
    }
    Shared state(cfg);
    const std::string base = "/primi-ball-balance/moveEnetOFK/" + cfg.camera;
    ev::BufferedPort<ev::AE> events;
    yarp::os::BufferedPort<Mono> modelImages, erosOutput;
    yarp::os::BufferedPort<Rgb> overlayOutput;
    yarp::os::BufferedPort<yarp::os::Bottle> modelReplies, skeletonOutput,
        velocityOutput, movenetSkeletonOutput, ofkSkeletonOutput;
    if (!events.open(base + "/events:i") ||
        !modelImages.open(base + "/model/img:o") ||
        !modelReplies.open(base + "/model/sklt:i") ||
        !erosOutput.open(base + "/eros:o") ||
        !overlayOutput.open(base + "/overlay:o") ||
        !skeletonOutput.open(base + "/skeleton:o") ||
        !velocityOutput.open(base + "/velocity:o") ||
        !movenetSkeletonOutput.open(base + "/movenet/skeleton:o") ||
        !ofkSkeletonOutput.open(base + "/ofk/skeleton:o")) {
        std::cerr << "Unable to open all YARP ports (name conflict?).\n";
        events.close(); modelImages.close(); modelReplies.close();
        erosOutput.close(); overlayOutput.close(); skeletonOutput.close();
        velocityOutput.close(); movenetSkeletonOutput.close();
        ofkSkeletonOutput.close();
        return 1;
    }
    if (!ev::ts_status) {
        std::cerr << "[WARN] event-driven reports individual event timestamps "
                     "disabled (VLIB_ENABLE_TS=OFF). SAE will use packet times; "
                     "optical-flow accuracy must be validated on live data.\n";
    }
    std::cout << "[noise] " << (cfg.noiseFilter ? "enabled" : "disabled")
              << "; refractory=" << cfg.noiseRefractoryMs << " ms"
              << "; spatial=" << cfg.noiseSpatialMs << " ms"
              << "; radius=" << cfg.noiseRadius << " px\n";
    if (cfg.noiseFilter && !ev::ts_status) {
        std::cerr << "[WARN] Noise filter uses packet-level timestamps; "
                     "spatial/refractory windows are approximate.\n";
    }
    std::cout << "PRIMI MoveEnetOFK online: " << cfg.camera
              << " camera, " << cfg.width << 'x' << cfg.height
              << ", f_net=" << cfg.fNet << ", f_flow=" << cfg.fFlow
              << ", f_vis=" << cfg.fVis << " Hz; LC=" << cfg.useLC << '\n'
              << "Event input: " << base << "/events:i\n"
              << "Skeleton output (f_vis): " << base << "/skeleton:o\n"
              << "MoveEnet detections (f_net): " << base << "/movenet/skeleton:o\n"
              << "OF+KF updates (f_flow): " << base << "/ofk/skeleton:o\n"
              << "Combined overlay (f_vis): " << base << "/overlay:o\n";

    std::thread acquisition(acquisitionLoop, std::ref(state), std::ref(events));
    std::thread requests(requestLoop, std::ref(state), std::ref(modelImages));
    std::thread responses(responseLoop, std::ref(state), std::ref(modelReplies),
                          std::ref(movenetSkeletonOutput));
    std::thread flow(flowLoop, std::ref(state), std::ref(ofkSkeletonOutput));
    std::thread visual(visualizationLoop, std::ref(state), std::ref(erosOutput),
                       std::ref(overlayOutput), std::ref(skeletonOutput),
                       std::ref(velocityOutput));

    auto lastStats = Clock::now();
    while (state.running.load() && !stopRequested.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (Clock::now() - lastStats < std::chrono::seconds(5)) continue;
        lastStats = Clock::now();
        std::cout << "[stats] packets=" << state.packets.load()
                  << " raw_events=" << state.rawEvents.load()
                  << " events_used=" << state.events.load()
                  << " noise_refractory=" << state.refractoryDrops.load()
                  << " noise_spatial=" << state.spatialDrops.load()
                  << " invalid=" << state.invalidEvents.load()
                  << " net_requests=" << state.requests.load()
                  << " net_replies=" << state.modelRepliesReceived.load()
                  << " net_results=" << state.detections.load()
                  << " flow_steps=" << state.flowSteps.load() << '\n';
    }
    state.running.store(false);
    events.interrupt();
    modelReplies.interrupt();
    acquisition.join(); requests.join(); responses.join(); flow.join(); visual.join();
    events.close(); modelImages.close(); modelReplies.close();
    erosOutput.close(); overlayOutput.close(); skeletonOutput.close();
    velocityOutput.close(); movenetSkeletonOutput.close();
    ofkSkeletonOutput.close();
    return 0;
}
