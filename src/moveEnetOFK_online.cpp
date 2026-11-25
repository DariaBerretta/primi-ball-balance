// PRIMI: online MoveEnet positions + event-based optical flow and Kalman fusion.
// The camera reader and robot controller run independently of this module.
#include <event-driven/core.h>
#include <event-driven/algs.h>
#include <event-driven/vis/filters.h>
#include <hpe-core/utility.h>
#include <hpe-core/representations.h>
#include <hpe-core/motion.h>
#include <hpe-core/fusion.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <functional>
#include <iostream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
using Mono = yarp::sig::ImageOf<yarp::sig::PixelMono>;
using Rgb = yarp::sig::ImageOf<yarp::sig::PixelRgb>;
using BottlePort = yarp::os::BufferedPort<yarp::os::Bottle>;

std::atomic<bool> stopping{false};
void onSignal(int) { stopping.store(true); }

constexpr std::array<int, 7> joints{{hpecore::head, hpecore::shoulderR,
    hpecore::shoulderL, hpecore::elbowR, hpecore::elbowL,
    hpecore::handR, hpecore::handL}};

bool upper(int id) {
    for (int j : joints) if (j == id) return true;
    return false;
}

struct Config {
    std::string camera = "left";
    int width = 640, height = 480, roi = 20, noiseRadius = 1;
    double fNet = 20, fFlow = 200, fVis = 30, threshold = 0.2;
    double timeout = 1, pu = 0.77, muD = 0.06, muV = 0.97;
    double refractoryMs = 1, spatialMs = 5;
    bool useLC = true, noise = true, showNet = true, showOFK = true;

    explicit Config(yarp::os::ResourceFinder &rf) {
        using yarp::os::Value;
        camera = rf.check("camera", Value("left")).asString();
        width = rf.check("width", Value(640)).asInt32();
        height = rf.check("height", Value(480)).asInt32();
        roi = rf.check("roi", Value(20)).asInt32();
        fNet = rf.check("f_net", Value(20.0)).asFloat64();
        fFlow = rf.check("f_flow", Value(200.0)).asFloat64();
        fVis = rf.check("f_vis", Value(30.0)).asFloat64();
        threshold = rf.check("confidence_th", Value(0.2)).asFloat64();
        timeout = rf.check("track_timeout", Value(1.0)).asFloat64();
        pu = rf.check("pu", Value(0.77)).asFloat64();
        muD = rf.check("muD", Value(0.06)).asFloat64();
        muV = rf.check("muV", Value(0.97)).asFloat64();
        useLC = rf.check("use_lc", Value(true)).asBool();
        noise = rf.check("noise_filter", Value(true)).asBool();
        refractoryMs = rf.check("noise_refractory_ms", Value(1.0)).asFloat64();
        spatialMs = rf.check("noise_spatial_ms", Value(5.0)).asFloat64();
        noiseRadius = rf.check("noise_radius", Value(1)).asInt32();
        showNet = rf.check("show_movenet", Value(true)).asBool();
        showOFK = rf.check("show_ofk", Value(true)).asBool();
    }

    bool valid() const {
        // Reject only values that would break allocation, indexing or timing.
        return (camera == "left" || camera == "right") &&
            width > 0 && height > 0 && roi >= 3 && roi < width &&
            roi < height && fNet > 0 && fFlow > 0 && fVis > 0 &&
            pu > 0 && muD > 0 && timeout > 0 &&
            std::isfinite(fNet) && std::isfinite(fFlow) &&
            std::isfinite(fVis) && std::isfinite(refractoryMs) &&
            std::isfinite(spatialMs) && refractoryMs >= 0 &&
            spatialMs >= 0 && noiseRadius >= 1 && noiseRadius <= 3;
    }
};

// Keep each worker close to its requested frequency without catching up in a burst.
struct Rate {
    Clock::duration step;
    Clock::time_point next = Clock::now();
    explicit Rate(double hz) : step(std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / hz))) {}
    void wait() {
        next += step;
        std::this_thread::sleep_until(next);
        if (next < Clock::now() - std::chrono::seconds(1)) next = Clock::now();
    }
};

struct Pending {
    int sequence;
    double time, sourceTime;
    Clock::time_point sent;
};

struct Shared {
    explicit Shared(const Config &config) : cfg(config) {
        eros.init(cfg.width, cfg.height, 7, 0.3);
        sae.init(cfg.width, cfg.height);
        if (cfg.noise) {
            filter.initialise(cfg.width, cfg.height);
            if (cfg.refractoryMs > 0)
                filter.use_temporal_filter(cfg.refractoryMs * 0.001);
            if (cfg.spatialMs > 0)
                filter.use_spatial_filter(cfg.spatialMs * 0.001, cfg.noiseRadius);
        }
        // Value-initialize hpe-core's history arrays before initializing LC.
        if (!tracker.initialise({cfg.pu, cfg.muD, cfg.muV, cfg.useLC ? 1.0 : 0.0}))
            throw std::runtime_error("Cannot initialize hpe-core Kalman filter");
        lastDetection.fill(-1.0);
        flow.prev_update_ts = 0;
    }

    Config cfg;
    ev::vNoiseFilter filter;           // Accessed by acquisition only.
    std::mutex surfacesMutex;        // Protects EROS, SAE and their timestamps.
    ev::EROS eros;
    hpecore::SAE sae;
    double time = 0, sourceTime = 0;

    std::mutex trackerMutex;         // Protects the fused and raw skeletons.
    hpecore::multiJointLatComp tracker{};
    hpecore::pwtripletvelocity flow{};
    hpecore::skeleton13 rawPose{};
    hpecore::confidence13 rawConf{}, confidence{};
    std::array<double, 13> lastDetection{};
    double rawTime = -1;

    std::mutex requestMutex;         // One pending GPU request, never a queue.
    std::optional<Pending> pending;
    int nextSequence = 0;
    bool inferenceTimedOut = false;  // Read by main only after joining workers.

    std::atomic<std::uint64_t> rawEvents{0}, usedEvents{0};
    std::atomic<std::uint64_t> requests{0}, replies{0}, results{0}, steps{0};
};

// Translate the hpe-core 13-joint message and mask unobserved/invalid joints.
// The Python sidecar always transmits one SKLT label followed by 39 values.
bool decode(const yarp::os::Bottle &msg, const Config &cfg,
            hpecore::skeleton13 &pose, hpecore::confidence13 &confidence) {
    if (msg.size() != 2 || msg.get(0).asString() != "SKLT") return false;
    const yarp::os::Bottle *values = msg.get(1).asList();
    if (!values || values->size() != 39) return false;
    for (int j = 0; j < 13; ++j) {
        const double x = values->get(2*j).asFloat64();
        const double y = values->get(2*j+1).asFloat64();
        const double c = values->get(26+j).asFloat64();
        if (upper(j) && c >= cfg.threshold && std::isfinite(c) &&
            std::isfinite(x) && std::isfinite(y) && x >= 0 && y >= 0 &&
            x < cfg.width && y < cfg.height) {
            pose[j] = {static_cast<float>(x), static_cast<float>(y)};
            confidence[j] = c;
        }
    }
    return true;
}

void publishPose(BottlePort &port, const char *label,
                 const hpecore::skeleton13 &pose,
                 const hpecore::confidence13 &confidence, int id, double time) {
    if (!port.getOutputCount()) return;
    yarp::os::Bottle &msg = port.prepare();
    msg.clear();
    msg.addString(label);
    yarp::os::Bottle &values = msg.addList();
    for (const hpecore::joint &joint : pose) {
        values.addFloat64(joint.u);
        values.addFloat64(joint.v);
    }
    for (double c : confidence) values.addFloat64(c);
    yarp::os::Stamp stamp(id, time);
    port.setEnvelope(stamp);
    port.write();
}

template<typename Pixel>
void publishImage(yarp::os::BufferedPort<yarp::sig::ImageOf<Pixel>> &port,
                  const cv::Mat &pixels, int id, double time) {
    if (!port.getOutputCount()) return;
    yarp::sig::ImageOf<Pixel> &image = port.prepare();
    image.resize(pixels.cols, pixels.rows);
    cv::Mat target(image.height(), image.width(), pixels.type(),
                   image.getRawImage(), image.getRowSize());
    pixels.copyTo(target);
    // The YARP installation requires an lvalue PortWriter, not a temporary.
    yarp::os::Stamp stamp(id, time);
    port.setEnvelope(stamp);
    port.write();
}

// readAll() is the same API used by EDPR-APRIL. Group by original packet
// identity so that SAE/LC retain packet timestamps if AE timestamps are off.
void acquire(Shared &s, ev::window<ev::AE> &input) {
    struct Event { int x, y; double time; };
    std::vector<Event> accepted;
    double epoch = 0, previousTime = 0;
    while (!stopping && input.isRunning()) {
        input.readAll(true);
        if (stopping || !input.isRunning()) break;
        ev::window<ev::AE>::iterator cursor = input.begin();
        const ev::window<ev::AE>::iterator end = input.end();
        while (cursor != end && !stopping) {
            const ev::window<ev::AE>::iterator first = cursor;
            const double sourceTime = cursor.timestamp();
            const double packetId = cursor.packetID();
            unsigned int lastTick = 0;
            do {
                if (ev::ts_status) lastTick = cursor->ts;
                ++cursor;
            } while (cursor != end && cursor.timestamp() == sourceTime &&
                     cursor.packetID() == packetId);

            // Reversing the event clock would invalidate Kalman history.
            if (!std::isfinite(sourceTime) || sourceTime <= 0 ||
                (previousTime > 0 && sourceTime + 0.001 < previousTime)) {
                std::cerr << "[ERROR] Invalid or reversed event timestamp\n";
                stopping = true;
                break;
            }
            previousTime = sourceTime;
            if (epoch == 0) epoch = sourceTime;
            const double time = 1.0 + sourceTime - epoch;
            accepted.clear();
            std::uint64_t received = 0;
            for (ev::window<ev::AE>::iterator it = first; it != cursor; ++it) {
                ++received;
                const ev::AE &event = *it;
                const int x = event.x, y = event.y;
                // Native filters index matrices directly: bound checking is essential.
                if (x < 0 || y < 0 || x >= s.cfg.width || y >= s.cfg.height)
                    continue;
                double eventTime = time;
                if (ev::ts_status) {
                    const double age = ev::deltaS(static_cast<int>(lastTick),
                                                   static_cast<int>(event.ts));
                    if (age >= 0 && age < 2) eventTime -= age;
                }
                if (s.cfg.noise && !s.filter.check(x, y, event.p, eventTime))
                    continue;
                accepted.push_back({x, y, eventTime});
            }
            // Filter outside the mutex; apply accepted events under one lock.
            {
                std::lock_guard<std::mutex> lock(s.surfacesMutex);
                for (const Event &event : accepted) {
                    s.eros.update(event.x, event.y);
                    s.sae.update(event.x, event.y, event.time);
                }
                s.time = time;
                s.sourceTime = sourceTime;
            }
            s.rawEvents += received;
            s.usedEvents += accepted.size();
        }
    }
    stopping = true;
}

// The GPU receives only one 8-bit EROS image per request. If it is still
// computing the previous frame, skip the new frame rather than queueing it.
void request(Shared &s, yarp::os::BufferedPort<Mono> &output) {
    Rate rate(s.cfg.fNet);
    while (!stopping) {
        if (output.getOutputCount()) {
            bool available = false;
            {
                std::lock_guard<std::mutex> lock(s.requestMutex);
                available = !s.pending;
                if (s.pending && Clock::now() - s.pending->sent >
                    std::chrono::seconds(10)) {
                    std::cerr << "[ERROR] MoveEnet response timed out\n";
                    s.inferenceTimedOut = true;
                    stopping = true;
                }
            }
            if (available && !stopping) {
                cv::Mat image;
                double time = 0, sourceTime = 0;
                {
                    std::lock_guard<std::mutex> lock(s.surfacesMutex);
                    if (s.time > 0) {
                        s.eros.getSurface().convertTo(image, CV_8U);
                        time = s.time;
                        sourceTime = s.sourceTime;
                    }
                }
                if (!image.empty()) {
                    // EDPR-APRIL network preprocessing: CV_8U + 5x5 blur,
                    // without min-max normalization or 255x rescaling.
                    cv::GaussianBlur(image, image, cv::Size(5, 5), 0);
                    int sequence;
                    {
                        std::lock_guard<std::mutex> lock(s.requestMutex);
                        sequence = ++s.nextSequence;
                        s.pending = Pending{sequence, time, sourceTime, Clock::now()};
                    }
                    Mono &target = output.prepare();
                    target.resize(image.cols, image.rows);
                    cv::Mat view(target.height(), target.width(), CV_8UC1,
                                 target.getRawImage(), target.getRowSize());
                    image.copyTo(view);
                    yarp::os::Stamp stamp(sequence, sourceTime);
                    output.setEnvelope(stamp);
                    output.write();
                    ++s.requests;
                }
            }
        }
        rate.wait();
    }
}

// Match the original EROS frame stamp before correcting the delayed Kalman
// state. Store raw network measurements separately for the red overlay.
void receive(Shared &s, BottlePort &input, BottlePort &rawOutput) {
    while (!stopping) {
        const yarp::os::Bottle *msg = input.read(true);
        if (!msg) break;
        yarp::os::Stamp stamp;
        if (!input.getEnvelope(stamp)) continue;
        Pending request;
        {
            std::lock_guard<std::mutex> lock(s.requestMutex);
            if (!s.pending || stamp.getCount() != s.pending->sequence ||
                std::abs(stamp.getTime() - s.pending->sourceTime) > 1e-4)
                continue;
            request = *s.pending;
            s.pending.reset();
        }
        hpecore::skeleton13 pose{};
        hpecore::confidence13 conf{};
        if (!decode(*msg, s.cfg, pose, conf)) continue;
        bool detected = false;
        {
            std::lock_guard<std::mutex> lock(s.trackerMutex);
            s.rawPose = pose;
            s.rawConf = conf;
            s.rawTime = request.time;
            for (int j : joints) {
                if (conf[j] == 0 || request.time <= s.lastDetection[j]) continue;
                if (!s.tracker.poseIsInitialised()) {
                    // Initialize once from the first valid full pose.
                    s.tracker.set(pose, request.time);
                    for (int k : joints) if (conf[k]) {
                        s.lastDetection[k] = request.time;
                        s.confidence[k] = conf[k];
                    }
                    detected = true;
                    break;
                }
                s.tracker.updateFromPosition(
                    static_cast<hpecore::jointName>(j), pose[j], request.time);
                s.lastDetection[j] = request.time;
                s.confidence[j] = conf[j];
                detected = true;
            }
        }
        ++s.replies;
        if (detected) ++s.results;
        publishPose(rawOutput, "SKLT", pose, conf, request.sequence, request.sourceTime);
    }
}

// Estimate only active joint ROIs directly from the shared SAE; no surface
// clone is made. The vector overload avoids the 1.4 factor in hpe-core's
// skeleton13 overload. Lock tracker then surfaces consistently.
void opticalFlow(Shared &s, BottlePort &output) {
    Rate rate(s.cfg.fFlow);
    double previousTime = 0;
    int sequence = 0;
    while (!stopping) {
        hpecore::skeleton13 pose{};
        hpecore::confidence13 conf{};
        double sourceTime = 0;
        bool updated = false;
        {
            std::lock_guard<std::mutex> trackerLock(s.trackerMutex);
            std::lock_guard<std::mutex> surfaceLock(s.surfacesMutex);
            const double time = s.time;
            if (time > previousTime) {
                previousTime = time;
                sourceTime = s.sourceTime;
                if (!s.tracker.poseIsInitialised()) {
                    s.flow.prev_update_ts = time;
                } else {
                    pose = s.tracker.query();
                    std::vector<hpecore::joint> positions;
                    std::vector<int> ids;
                    positions.reserve(joints.size());
                    ids.reserve(joints.size());
                    for (int j : joints) {
                        if (s.lastDetection[j] < 0 ||
                            time <= s.lastDetection[j] ||
                            time - s.lastDetection[j] > s.cfg.timeout) continue;
                        positions.push_back(pose[j]);
                        ids.push_back(j);
                    }
                    const std::vector<hpecore::jDot> velocities =
                        s.flow.multi_area_velocity(s.sae.getSurface(),
                                                   time, positions, s.cfg.roi);
                    hpecore::skeleton13_vel allVel{};
                    for (std::size_t i = 0; i < ids.size(); ++i) {
                        allVel[ids[i]] = velocities[i];
                        s.tracker.updateFromVelocity(
                            static_cast<hpecore::jointName>(ids[i]),
                            velocities[i], time);
                    }
                    s.tracker.setVelocity(allVel);
                    pose = s.tracker.query();
                    for (int j = 0; j < 13; ++j) {
                        if (s.lastDetection[j] < 0 || !upper(j) ||
                            time - s.lastDetection[j] > s.cfg.timeout)
                            pose[j] = {0, 0};
                        else conf[j] = s.confidence[j];
                    }
                    ++s.steps;
                    updated = true;
                }
            }
        }
        if (updated)
            publishPose(output, "SKLT", pose, conf, ++sequence, sourceTime);
        rate.wait();
    }
}

// Draw the original neural skeleton as red circles and the OF+KF state as
// green crosses. The hpe-core tracker has only seven meaningful joints.
void drawPose(cv::Mat &image, const hpecore::skeleton13 &pose,
              const hpecore::confidence13 &conf, const Config &cfg,
              const cv::Scalar &color, bool circles) {
    const auto visible = [&](int j) {
        return conf[j] > 0 && std::isfinite(pose[j].u) &&
            std::isfinite(pose[j].v) && pose[j].u >= 0 && pose[j].v >= 0 &&
            pose[j].u < cfg.width && pose[j].v < cfg.height;
    };
    const auto point = [&](int j) {
        return cv::Point(cvRound(pose[j].u), cvRound(pose[j].v));
    };
    constexpr std::array<std::array<int, 2>, 5> edges{{
        {{hpecore::shoulderR, hpecore::shoulderL}},
        {{hpecore::shoulderR, hpecore::elbowR}},
        {{hpecore::elbowR, hpecore::handR}},
        {{hpecore::shoulderL, hpecore::elbowL}},
        {{hpecore::elbowL, hpecore::handL}}
    }};
    for (const std::array<int, 2> &edge : edges)
        if (visible(edge[0]) && visible(edge[1]))
            cv::line(image, point(edge[0]), point(edge[1]), color, 2);
    if (visible(hpecore::head) && visible(hpecore::shoulderR) &&
        visible(hpecore::shoulderL)) {
        const cv::Point neck((point(hpecore::shoulderR).x +
                              point(hpecore::shoulderL).x) / 2,
                             (point(hpecore::shoulderR).y +
                              point(hpecore::shoulderL).y) / 2);
        cv::line(image, point(hpecore::head), neck, color, 2);
    }
    for (int j : joints) if (visible(j)) {
        if (circles) cv::circle(image, point(j), 5, color, 2);
        else cv::drawMarker(image, point(j), color, cv::MARKER_CROSS, 10, 2);
    }
}

// Visualization is independent of the network and optical-flow frequencies.
// EDPR-APRIL displays EROS using 9x9 blur + min-max normalization, whereas
// MoveEnet receives an unnormalized 5x5-blurred 8-bit image.
void visualize(Shared &s, yarp::os::BufferedPort<Mono> &erosPort,
               yarp::os::BufferedPort<Rgb> &overlayPort,
               BottlePort &posePort, BottlePort &velocityPort) {
    Rate rate(s.cfg.fVis);
    int sequence = 0;
    while (!stopping) {
        cv::Mat image;
        double time = 0, sourceTime = 0;
        {
            std::lock_guard<std::mutex> lock(s.surfacesMutex);
            time = s.time;
            sourceTime = s.sourceTime;
            if (time > 0 && (erosPort.getOutputCount() ||
                             overlayPort.getOutputCount()))
                s.eros.getSurface().convertTo(image, CV_8U);
        }
        if (time > 0) {
            hpecore::skeleton13 pose{}, raw{};
            hpecore::skeleton13_vel velocity{};
            hpecore::confidence13 conf{}, rawConf{};
            {
                std::lock_guard<std::mutex> lock(s.trackerMutex);
                if (s.tracker.poseIsInitialised()) {
                    pose = s.tracker.query();
                    velocity = s.tracker.queryVelocity();
                    for (int j = 0; j < 13; ++j) {
                        if (!upper(j) || s.lastDetection[j] < 0 ||
                            time - s.lastDetection[j] > s.cfg.timeout) {
                            pose[j] = {0, 0};
                            velocity[j] = {0, 0};
                        } else conf[j] = s.confidence[j];
                    }
                }
                if (s.rawTime >= 0 && s.rawTime <= time &&
                    time - s.rawTime <= s.cfg.timeout) {
                    raw = s.rawPose;
                    rawConf = s.rawConf;
                }
            }
            const int id = ++sequence;
            publishPose(posePort, "SKLT", pose, conf, id, sourceTime);
            publishPose(velocityPort, "JVEL", velocity, conf, id, sourceTime);
            if (!image.empty()) {
                cv::GaussianBlur(image, image, cv::Size(5, 5), 0);
                // cv::normalize(image, image, 0, 255, cv::NORM_MINMAX);
                publishImage(erosPort, image, id, sourceTime);
                if (overlayPort.getOutputCount()) {
                    cv::Mat bgr;
                    cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
                    if (s.cfg.showOFK)
                        drawPose(bgr, pose, conf, s.cfg, {0, 255, 0}, false);
                    if (s.cfg.showNet)
                        drawPose(bgr, raw, rawConf, s.cfg, {0, 0, 255}, true);
                    cv::Mat rgb;
                    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
                    publishImage(overlayPort, rgb, id, sourceTime);
                }
            }
        }
        rate.wait();
    }
}
}  // namespace

int main(int argc, char **argv) {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    const Config cfg(rf);
    if (!cfg.valid()) {
        std::cerr << "Invalid MoveEnetOFK configuration\n";
        return 1;
    }
    yarp::os::Network network;
    if (!yarp::os::Network::checkNetwork(2.0)) return 1;
    Shared state(cfg);
    const std::string base = "/primi-ball-balance/moveEnetOFK/" + cfg.camera;
    ev::window<ev::AE> events;
    yarp::os::BufferedPort<Mono> modelImages, erosImages;
    yarp::os::BufferedPort<Rgb> overlayImages;
    BottlePort modelReplies, poseImages, velocityImages, rawPoses, flowPoses;
    if (!events.open(base + "/events:i") ||
        !modelImages.open(base + "/model/img:o") ||
        !modelReplies.open(base + "/model/sklt:i") ||
        !erosImages.open(base + "/eros:o") ||
        !overlayImages.open(base + "/overlay:o") ||
        !poseImages.open(base + "/skeleton:o") ||
        !velocityImages.open(base + "/velocity:o") ||
        !rawPoses.open(base + "/movenet/skeleton:o") ||
        !flowPoses.open(base + "/ofk/skeleton:o")) {
        std::cerr << "Cannot open MoveEnetOFK YARP ports\n";
        events.stop();
        return 1;
    }
    if (!ev::ts_status)
        std::cerr << "[WARN] Individual AE timestamps disabled; SAE uses packet time\n";
    std::cout << "[noise] " << (cfg.noise ? "enabled" : "disabled")
              << "; refractory=" << cfg.refractoryMs << " ms; spatial="
              << cfg.spatialMs << " ms; radius=" << cfg.noiseRadius << "\n";

    // The five workers do not block on each other's YARP writes.
    std::thread camera(acquire, std::ref(state), std::ref(events));
    std::thread networkRequest(request, std::ref(state), std::ref(modelImages));
    std::thread networkReply(receive, std::ref(state), std::ref(modelReplies),
                             std::ref(rawPoses));
    std::thread flow(opticalFlow, std::ref(state), std::ref(flowPoses));
    std::thread display(visualize, std::ref(state), std::ref(erosImages),
                        std::ref(overlayImages), std::ref(poseImages),
                        std::ref(velocityImages));
    Clock::time_point lastReport = Clock::now();
    while (!stopping) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (Clock::now() - lastReport < std::chrono::seconds(5)) continue;
        lastReport = Clock::now();
        std::cout << "[stats] raw_events=" << state.rawEvents <<
            " events_used=" << state.usedEvents <<
            " events_dropped=" << state.rawEvents - state.usedEvents <<
            " net_requests=" << state.requests <<
            " net_replies=" << state.replies <<
            " net_results=" << state.results <<
            " flow_steps=" << state.steps << '\n';
    }
    events.stop();
    modelReplies.interrupt();
    camera.join(); networkRequest.join(); networkReply.join();
    flow.join(); display.join();
    return state.inferenceTimedOut ? 1 : 0;
}
