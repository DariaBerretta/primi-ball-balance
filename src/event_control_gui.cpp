// Local stereo event viewer. Acquisition and camera-hardware control are separate.
#include <event-driven/core.h>
#include <event-driven/algs.h>
#include <event-driven/vis/filters.h>
#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Bottle.h>

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QMainWindow>
#include <QPixmap>
#include <QPushButton>
#include <QSlider>
#include <QSizePolicy>
#include <QSpinBox>
#include <QStatusBar>
#include <QTimer>
#include <QVBoxLayout>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {
constexpr int width = 640, height = 480;
const char* sources[2] = {
    "/primi-ball-balance/eventCam/left/raw:o",
    "/primi-ball-balance/eventCam/right/raw:o"
};
const char* inputs[2] = {
    "/primi-ball-balance/eventControl/left/events:i",
    "/primi-ball-balance/eventControl/right/events:i"
};
using Clock = std::chrono::steady_clock;

struct DisplayEvent {
    uint16_t x, y;
    bool polarity;
};
struct DisplayPacket {
    Clock::time_point received;
    std::vector<DisplayEvent> events;
};

struct Camera {
    ev::BufferedPort<ev::AE> input;
    std::thread worker;
    std::atomic<bool> running{false};
    std::atomic<int> kernel{7}, decayMilli{300}, mode{0}, windowMs{30};
    std::atomic<bool> denoise{false};
    std::mutex mutex;
    std::deque<DisplayPacket> history;
    std::size_t historyCount{0};
    ev::EROS eros;
    ev::SAE sae;
    ev::TOS tos;
    ev::vNoiseFilter filter;
    int currentKernel{7}, currentDecay{300}, currentMode{0};
    bool currentDenoise{false};
    double lastEventTime{0}, eventRate{0}, acceptedPercent{100};
    Clock::time_point lastPacket{};

    Camera() { reset(); }

    void reset() {
        eros.init(width, height, currentKernel, currentDecay / 1000.0);
        sae.init(width, height);
        tos.init(width, height, currentKernel, 2.0);
        filter = ev::vNoiseFilter();
        filter.initialise(width, height);
        filter.use_spatial_filter(0.005, 1);
        lastEventTime = 0;
    }

    bool start(int side) {
        if (!input.open(inputs[side])) return false;
        running = true;
        worker = std::thread([this, side] {
            bool sourceAvailable = false;
            auto nextConnect = Clock::time_point{};
            auto lastRate = Clock::now();
            uint64_t seen = 0, kept = 0;
            while (running.load()) {
                auto now = Clock::now();
                // Connect only when the source becomes available. Do not repeatedly
                // connect on transient shmem status checks.
                if (now >= nextConnect) {
                    const bool available = yarp::os::Network::exists(sources[side]);
                    if (available && !sourceAvailable)
                        yarp::os::Network::connect(sources[side], inputs[side], "shmem");
                    sourceAvailable = available;
                    nextConnect = now + std::chrono::seconds(2);
                }
                auto* packet = input.read(false);
                if (packet && packet->size()) {
                    // YARP packet envelope is the reference time. With enabled
                    // event timestamps, recover each event's relative offset.
                    double packetTime = packet->envelope().getTime();
                    if (packetTime <= 0)
                        packetTime = std::chrono::duration<double>(now.time_since_epoch()).count();
                    int lastTick = 0;
                    for (const auto& event : *packet) lastTick = event.ts;
                    DisplayPacket display{now, {}};
                    const std::size_t stride = std::max<std::size_t>(1, packet->size() / 2000);
                    display.events.reserve(packet->size() / stride + 1);

                    std::lock_guard<std::mutex> lock(mutex);
                    int k = kernel.load(), d = decayMilli.load(), m = mode.load();
                    bool noise = denoise.load();
                    if (k != currentKernel || d != currentDecay ||
                        m != currentMode || noise != currentDenoise) {
                        currentKernel = k;
                        currentDecay = d;
                        currentMode = m;
                        currentDenoise = noise;
                        reset();
                    }
                    std::size_t i = 0;
                    for (const auto& e : *packet) {
                        const int x = e.x, y = e.y;
                        if (x < 0 || x >= width || y < 0 || y >= height) continue;
                        if (i++ % stride == 0)
                            display.events.push_back({static_cast<uint16_t>(x),
                                static_cast<uint16_t>(y), bool(e.p)});
                        ++seen;
                        double t = packetTime;
                        if (ev::ts_status) {
                            const double age = ev::deltaS(lastTick, static_cast<int>(e.ts));
                            if (age >= 0 && age < 2.0) t -= age;
                        }
                        if (noise && !filter.check(x, y, e.p, t)) continue;
                        ++kept;
                        lastEventTime = std::max(lastEventTime, t);
                        if (m == 0) eros.update(x, y);
                        else if (m == 1) sae.update(x, y, t);
                        else tos.update(x, y);
                    }
                    historyCount += display.events.size();
                    history.push_back(std::move(display));
                    // Bound display memory even at very high event rates.
                    while (historyCount > 250000 && !history.empty()) {
                        historyCount -= history.front().events.size();
                        history.pop_front();
                    }
                    lastPacket = now;
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
                now = Clock::now();
                if (now - lastRate >= std::chrono::milliseconds(500)) {
                    std::lock_guard<std::mutex> lock(mutex);
                    const double dt = std::chrono::duration<double>(now - lastRate).count();
                    eventRate = seen / dt;
                    acceptedPercent = seen ? 100.0 * kept / seen : 100.0;
                    seen = kept = 0;
                    lastRate = now;
                }
            }
        });
        return true;
    }

    void stop() {
        running = false;
        input.interrupt();
        if (worker.joinable()) worker.join();
        input.close();
    }

    void snapshot(int rawMode, QImage& rawImage, QImage& representationImage,
                  double& rate, double& accepted, bool& receiving) {
        auto now = Clock::now();
        cv::Mat raw(height, width, CV_8UC3,
                    rawMode == 2 ? cv::Scalar(0, 0, 0) : cv::Scalar(255, 255, 255));
        cv::Mat processed;
        {
            std::lock_guard<std::mutex> lock(mutex);
            const auto cutoff = now - std::chrono::milliseconds(windowMs.load());
            while (!history.empty() && history.front().received < cutoff) {
                historyCount -= history.front().events.size();
                history.pop_front();
            }
            for (const auto& pkt : history)
                for (const auto& e : pkt.events) {
                    auto& pixel = raw.at<cv::Vec3b>(e.y, e.x);
                    if (rawMode == 0)
                        pixel = e.polarity ? cv::Vec3b(40, 40, 220)
                                           : cv::Vec3b(220, 110, 30);
                    else if (rawMode == 1)
                        pixel = cv::Vec3b(0, 0, 0);
                    else
                        pixel = cv::Vec3b(255, 255, 255);
                }
            if (currentMode == 0) {
                eros.getSurface().convertTo(processed, CV_8U, 255.0);
            } else if (currentMode == 1) {
                // SAE is a timestamp surface; only its DISPLAY mapping uses
                // exponential temporal decay. This does not modify ev::SAE.
                cv::Mat stamps = sae.getSurface();
                cv::Mat age = lastEventTime - stamps;
                cv::max(age, cv::Scalar(0), age);
                cv::exp(-age / 0.1, age);
                age.setTo(0, stamps == 0);
                age.convertTo(processed, CV_8U, 255.0);
            } else {
                tos.getSurface().convertTo(processed, CV_8U, 255.0);
            }
            rate = eventRate;
            accepted = acceptedPercent;
            receiving = lastPacket != Clock::time_point{} &&
                        now - lastPacket < std::chrono::seconds(2);
        }
        cv::cvtColor(raw, raw, cv::COLOR_BGR2RGB);
        rawImage = QImage(raw.data, width, height, int(raw.step), QImage::Format_RGB888).copy();
        representationImage = QImage(processed.data, width, height, int(processed.step),
                                     QImage::Format_Grayscale8).copy();
    }
};

QWidget* panel(const QString& title, QLabel*& view) {
    auto* widget = new QWidget;
    auto* layout = new QVBoxLayout(widget);
    layout->setContentsMargins(4, 4, 4, 4);
    auto* name = new QLabel(title);
    name->setAlignment(Qt::AlignCenter);
    view = new QLabel;
    view->setAlignment(Qt::AlignCenter);
    view->setMinimumSize(240, 180);
    view->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    layout->addWidget(name);
    layout->addWidget(view, 1);
    return widget;
}

QWidget* slider(int lo, int hi, int initial, QSlider*& control) {
    auto* widget = new QWidget;
    auto* row = new QHBoxLayout(widget);
    row->setContentsMargins(0, 0, 0, 0);
    control = new QSlider(Qt::Horizontal);
    control->setRange(lo, hi);
    control->setValue(initial);
    auto* number = new QSpinBox;
    number->setRange(lo, hi);
    number->setValue(initial);
    QObject::connect(control, &QSlider::valueChanged, number, &QSpinBox::setValue);
    QObject::connect(number, QOverload<int>::of(&QSpinBox::valueChanged),
                     control, &QSlider::setValue);
    row->addWidget(control, 1);
    row->addWidget(number);
    return widget;
}

class Window : public QMainWindow {
    Camera cameras[2];
    QLabel *rawViews[2]{}, *processedViews[2]{};
    QComboBox *rawModes[2]{}, *representations[2]{};
    QCheckBox *noise[2]{}, *linkHardware{}, *linkEros{};
    QSlider *sensitivity[2]{}, *refractory[2]{}, *kernel[2]{}, *decay[2]{}, *accumulation{};
    QPushButton* apply{};
    yarp::os::RpcClient rpc;
    std::future<QString> pending;
    QString rpcMessage;
    Clock::time_point rpcMessageUntil{};

public:
    Window() {
        setWindowTitle("PRIMI - Event Cameras and EROS");
        resize(1300, 960);
        auto* central = new QWidget;
        auto* root = new QVBoxLayout(central);
        auto* grid = new QGridLayout;
        for (int s = 0; s < 2; ++s) {
            grid->addWidget(panel(s ? "RIGHT RAW" : "LEFT RAW", rawViews[s]), 0, s);
            grid->addWidget(panel(s ? "RIGHT REPRESENTATION" : "LEFT REPRESENTATION",
                                  processedViews[s]), 1, s);
        }
        grid->setRowStretch(0, 1);
        grid->setRowStretch(1, 1);
        root->addLayout(grid, 1);

        auto* settings = new QHBoxLayout;
        auto* hardwareBox = new QGroupBox("Camera hardware (Ultrascale I2C RPC)");
        auto* hardware = new QFormLayout(hardwareBox);
        linkHardware = new QCheckBox("Link cameras");
        linkHardware->setChecked(true);
        hardware->addRow(linkHardware);
        for (int s = 0; s < 2; ++s) {
            hardware->addRow(s ? "Right sensitivity" : "Left sensitivity",
                             slider(10, 90, 70, sensitivity[s]));
            hardware->addRow(s ? "Right refractory bias" : "Left refractory bias",
                             slider(1, 99, 1, refractory[s]));
        }
        apply = new QPushButton("Apply camera settings");
        hardware->addRow(apply);
        settings->addWidget(hardwareBox);

        auto* processingBox = new QGroupBox("Local representation and filtering");
        auto* processing = new QFormLayout(processingBox);
        linkEros = new QCheckBox("Link EROS parameters");
        linkEros->setChecked(true);
        processing->addRow(linkEros);
        for (int s = 0; s < 2; ++s) {
            auto* modes = new QWidget;
            auto* row = new QHBoxLayout(modes);
            row->setContentsMargins(0, 0, 0, 0);
            rawModes[s] = new QComboBox;
            rawModes[s]->addItems({"Polarity", "Black on white", "White on black"});
            representations[s] = new QComboBox;
            representations[s]->addItems({"EROS", "SAE", "TOS"});
            row->addWidget(rawModes[s]);
            row->addWidget(representations[s]);
            processing->addRow(s ? "Right display" : "Left display", modes);
            noise[s] = new QCheckBox("Salt-and-pepper filter before representation");
            processing->addRow(noise[s]);
            processing->addRow(s ? "Right kernel (odd)" : "Left kernel (odd)",
                               slider(1, 15, 7, kernel[s]));
            processing->addRow(s ? "Right EROS decay x1000" : "Left EROS decay x1000",
                               slider(1, 999, 300, decay[s]));
        }
        processing->addRow("Raw display window (ms)", slider(5, 200, 30, accumulation));
        settings->addWidget(processingBox);
        root->addLayout(settings);
        setCentralWidget(central);

        for (int s = 0; s < 2; ++s) {
            const int other = 1 - s;
            connect(sensitivity[s], &QSlider::valueChanged, this, [=](int n) {
                if (linkHardware->isChecked()) sensitivity[other]->setValue(n);
            });
            connect(refractory[s], &QSlider::valueChanged, this, [=](int n) {
                if (linkHardware->isChecked()) refractory[other]->setValue(n);
            });
            connect(kernel[s], &QSlider::valueChanged, this, [=](int n) {
                if (!(n & 1)) {
                    kernel[s]->setValue(n > cameras[s].kernel.load() ? n + 1 : n - 1);
                    return;
                }
                if (linkEros->isChecked()) kernel[other]->setValue(n);
                cameras[s].kernel = n;
            });
            connect(decay[s], &QSlider::valueChanged, this, [=](int n) {
                if (linkEros->isChecked()) decay[other]->setValue(n);
                cameras[s].decayMilli = n;
            });
            connect(representations[s], QOverload<int>::of(&QComboBox::currentIndexChanged),
                    this, [=](int n) { cameras[s].mode = n; });
            connect(noise[s], &QCheckBox::toggled, this,
                    [=](bool checked) { cameras[s].denoise = checked; });
            if (!cameras[s].start(s))
                statusBar()->showMessage(QString("Cannot open event input: ") + inputs[s]);
        }
        connect(accumulation, &QSlider::valueChanged, this, [this](int n) {
            for (auto& c : cameras) c.windowMs = n;
        });
        rpc.open("/primi-ball-balance/eventControl/rpc:o");
        connect(apply, &QPushButton::clicked, this, [this] {
            if (pending.valid()) return;
            const int values[2][2] = {{sensitivity[0]->value(), refractory[0]->value()},
                                      {sensitivity[1]->value(), refractory[1]->value()}};
            apply->setEnabled(false);
            pending = std::async(std::launch::async, [this, values] {
                if (!yarp::os::Network::connect(rpc.getName(), "/primi-ball-balance/cameraControl/rpc"))
                    return QString("Camera RPC unavailable; start camera_control on Ultrascale.");
                int accepted = 0;
                for (int s = 0; s < 2; ++s)
                    for (int p = 0; p < 2; ++p) {
                        yarp::os::Bottle request, response;
                        request.addString("set");
                        request.addString(p ? "refractory" : "sensitivity");
                        request.addString(s ? "right" : "left");
                        request.addInt32(values[s][p]);
                        if (rpc.write(request, response) && response.size() &&
                            response.get(0).asString() == "ok") ++accepted;
                    }
                return QString("Camera RPC acknowledged: %1/4 (inspect Ultrascale readback).").arg(accepted);
            });
        });
        auto* timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, [this] {
            QString summary;
            for (int s = 0; s < 2; ++s) {
                QImage raw, processed;
                double rate, accepted;
                bool receiving;
                cameras[s].snapshot(rawModes[s]->currentIndex(), raw, processed,
                                    rate, accepted, receiving);
                rawViews[s]->setPixmap(QPixmap::fromImage(raw).scaled(rawViews[s]->size(),
                                     Qt::KeepAspectRatio, Qt::FastTransformation));
                processedViews[s]->setPixmap(QPixmap::fromImage(processed).scaled(
                    processedViews[s]->size(), Qt::KeepAspectRatio, Qt::FastTransformation));
                summary += QString("%1: %2, %3 kEv/s, %4% accepted  ")
                    .arg(s ? "Right" : "Left").arg(receiving ? "live" : "no data")
                    .arg(rate / 1000.0, 0, 'f', 1).arg(accepted, 0, 'f', 1);
            }
            if (pending.valid() && pending.wait_for(std::chrono::seconds(0)) ==
                                   std::future_status::ready) {
                rpcMessage = pending.get();
                rpcMessageUntil = Clock::now() + std::chrono::seconds(8);
                apply->setEnabled(true);
            }
            if (Clock::now() < rpcMessageUntil) summary += " | " + rpcMessage;
            statusBar()->showMessage(summary);
        });
        timer->start(33);
    }
    ~Window() override {
        for (auto& c : cameras) c.stop();
        if (pending.valid()) pending.wait();
        rpc.close();
    }
};
} // namespace

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    yarp::os::Network yarp;
    if (!yarp::os::Network::checkNetwork(2.0)) return 1;
    Window window;
    window.show();
    return app.exec();
}
