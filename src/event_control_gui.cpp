// Stereo event-camera viewer and live EROS tuning for PRIMI.
// The GUI is a LOCAL consumer. It never opens the robot's hardware event ports.
#include <event-driven/core.h>
#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Bottle.h>

#include <QApplication>
#include <QCheckBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QStatusBar>
#include <QPixmap>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QTimer>
#include <QVBoxLayout>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace {
constexpr int WIDTH = 640;
constexpr int HEIGHT = 480;
const char *SOURCES[2] = {
    "/primi-ball-balance/eventCam/left/raw:o",
    "/primi-ball-balance/eventCam/right/raw:o"
};
const char *INPUTS[2] = {
    "/primi-ball-balance/eventControl/left/events:i",
    "/primi-ball-balance/eventControl/right/events:i"
};

struct Camera {
    ev::BufferedPort<ev::AE> input;
    std::thread worker;
    std::atomic<bool> running{true};
    std::atomic<int> kernel{7};
    std::atomic<double> decay{0.3};
    std::atomic<int> windowMs{30};
    std::atomic<unsigned> settingsVersion{0};
    std::mutex mutex;
    cv::Mat raw{HEIGHT, WIDTH, CV_8UC3, cv::Scalar(255, 255, 255)};
    cv::Mat eros{HEIGHT + 6, WIDTH + 6, CV_64F, cv::Scalar(0)};
    int activeKernel{7};
    double rate{0};
    bool connected{false};

    void start(int side) {
    if (!input.open(INPUTS[side])) return;
    yarp::os::Network::connect(SOURCES[side], INPUTS[side], "shmem");
        worker = std::thread([this, side] {
            using clock = std::chrono::steady_clock;
            auto lastFrame = clock::now();
            auto lastRate = lastFrame;
            auto lastConnect = lastFrame - std::chrono::seconds(5);
            uint64_t count = 0;
            unsigned version = ~0u;
            int k = 7;
            double factor = 1.0;
            while (running) {
                const auto now = clock::now();
            if (now - lastConnect >= std::chrono::seconds(1)) {
                if (!connected) {
                    yarp::os::Network::connect(
                        SOURCES[side], INPUTS[side], "shmem"
                    );
                }
                connected = true;
                lastConnect = now;
            }
                auto *packet = input.read(false);
                if (packet) {
                    connected = true;
                    const int requestedK = kernel.load();
                    const double requestedDecay = decay.load();
                    const unsigned requestedVersion = settingsVersion.load();
                    std::lock_guard<std::mutex> lock(mutex);
                    if (version != requestedVersion) {
                        k = requestedK;
                        factor = std::pow(requestedDecay, 1.0 / k);
                        eros = cv::Mat(HEIGHT + k - 1, WIDTH + k - 1,
                                       CV_64F, cv::Scalar(0));
                        activeKernel = k;
                        version = requestedVersion;
                    }
                    if (now - lastFrame >= std::chrono::milliseconds(windowMs.load())) {
                        raw.setTo(cv::Scalar(255, 255, 255));
                        lastFrame = now;
                    }
                    for (const auto &e : *packet) {
                        const int x = e.x, y = e.y;
                        if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) continue;
                        raw.at<cv::Vec3b>(y, x) = e.p
                            ? cv::Vec3b(40, 40, 220) : cv::Vec3b(220, 110, 30);
                        // Same EROS spatial update as event-driven's ev::EROS:
                        // padded ROI centered on the event; pixel becomes 1.
                        eros(cv::Rect(x, y, k, k)) *= factor;
                        eros.at<double>(y + k / 2, x + k / 2) = 1.0;
                        ++count;
                    }
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
                if (now - lastRate >= std::chrono::milliseconds(500)) {
                    const double seconds = std::chrono::duration<double>(now - lastRate).count();
                    std::lock_guard<std::mutex> lock(mutex);
                    rate = count / seconds;
                    lastRate = now;
                    count = 0;
                }
            }
        });
    }

    void stop() {
        running = false;
        input.interrupt();
        if (worker.joinable()) worker.join();
        input.close();
    }

    void snapshot(QImage &rawImage, QImage &erosImage, double &rateValue, bool &online) {
        cv::Mat rawCopy, erosion;
        {
            std::lock_guard<std::mutex> lock(mutex);
            rawCopy = raw.clone();
            const int border = activeKernel / 2;
            erosion = eros(cv::Rect(border, border, WIDTH, HEIGHT)).clone();
            rateValue = rate;
            online = connected;
        }
        cv::cvtColor(rawCopy, rawCopy, cv::COLOR_BGR2RGB);
        rawImage = QImage(rawCopy.data, WIDTH, HEIGHT, static_cast<int>(rawCopy.step),
                          QImage::Format_RGB888).copy();
        cv::Mat grey;
        erosion.convertTo(grey, CV_8U, 255.0);
        erosImage = QImage(grey.data, WIDTH, HEIGHT, static_cast<int>(grey.step),
                           QImage::Format_Grayscale8).copy();
    }
};

QWidget *makePanel(const QString &title, QLabel *&image) {
    auto *panel = new QWidget;
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(4, 4, 4, 4);
    auto *caption = new QLabel(title);
    caption->setAlignment(Qt::AlignCenter);
    image = new QLabel;
    image->setAlignment(Qt::AlignCenter);
    image->setMinimumSize(320, 240);
    image->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    layout->addWidget(caption);
    layout->addWidget(image, 1);
    return panel;
}

QSpinBox *intControl(int min, int max, int value) {
    auto *box = new QSpinBox;
    box->setRange(min, max);
    box->setValue(value);
    return box;
}

QDoubleSpinBox *doubleControl(double min, double max, double step, double value) {
    auto *box = new QDoubleSpinBox;
    box->setRange(min, max);
    box->setSingleStep(step);
    box->setDecimals(3);
    box->setValue(value);
    return box;
}

// A labelled slider synchronized with its numerical spin box.
QWidget *sliderControl(int low, int high, int initial, QSlider *&slider) {
    auto *widget = new QWidget;
    auto *row = new QHBoxLayout(widget);
    row->setContentsMargins(0, 0, 0, 0);
    slider = new QSlider(Qt::Horizontal);
    slider->setRange(low, high);
    slider->setValue(initial);
    auto *spin = intControl(low, high, initial);
    QObject::connect(slider, &QSlider::valueChanged, spin, &QSpinBox::setValue);
    QObject::connect(spin, QOverload<int>::of(&QSpinBox::valueChanged), slider, &QSlider::setValue);
    row->addWidget(slider, 1);
    row->addWidget(spin);
    return widget;
}

class Window : public QMainWindow {
    Camera cameras[2];
    QLabel *rawViews[2]{}, *erosViews[2]{};
    QLabel *state{};
    QSlider *sensitivity[2]{}, *refractory[2]{}, *kernel[2]{}, *decay[2]{};
    QSlider *accumulation{};
    QCheckBox *linkHardware{}, *linkEros{};
    yarp::os::RpcClient rpc;

    bool command(const char *setting, int side, int value) {
        if (rpc.getOutputCount() == 0 &&
            !yarp::os::Network::connect(rpc.getName(), "/zynqGrabber/control/rpc"))
            return false;
        yarp::os::Bottle request, reply;
        request.addString("set");
        request.addString(setting);
        request.addString(side == 0 ? "left" : "right");
        request.addInt32(value);
        return rpc.write(request, reply) && reply.size() > 0 &&
               reply.get(0).asString() == "ok";
    }

public:
    Window() {
        setWindowTitle("PRIMI - Event Cameras and EROS");
        resize(1350, 1050);
        auto *central = new QWidget;
        auto *outer = new QVBoxLayout(central);
        auto *images = new QWidget;
        auto *grid = new QGridLayout(images);
        grid->setSpacing(6);
        grid->addWidget(makePanel("LEFT - RAW EVENTS", rawViews[0]), 0, 0);
        grid->addWidget(makePanel("RIGHT - RAW EVENTS", rawViews[1]), 0, 1);
        grid->addWidget(makePanel("LEFT - EROS", erosViews[0]), 1, 0);
        grid->addWidget(makePanel("RIGHT - EROS", erosViews[1]), 1, 1);
        grid->setRowStretch(0, 1);
        grid->setRowStretch(1, 1);
        outer->addWidget(images, 1);

        auto *settings = new QWidget;
        auto *columns = new QHBoxLayout(settings);
        auto *hardware = new QGroupBox("Camera hardware (live RPC)");
        auto *hardwareLayout = new QFormLayout(hardware);
        linkHardware = new QCheckBox("Link left and right");
        linkHardware->setChecked(true);
        hardwareLayout->addRow(linkHardware);
        for (int s = 0; s < 2; ++s) {
            hardwareLayout->addRow(s == 0 ? "Left sensitivity" : "Right sensitivity",
                                   sliderControl(10, 90, 70, sensitivity[s]));
            hardwareLayout->addRow(s == 0 ? "Left refractory bias" : "Right refractory bias",
                                   sliderControl(1, 99, 1, refractory[s]));
        }
        auto *apply = new QPushButton("Apply camera settings");
        hardwareLayout->addRow(apply);
        columns->addWidget(hardware);

        auto *erosGroup = new QGroupBox("EROS and display (live, local)");
        auto *erosLayout = new QFormLayout(erosGroup);
        linkEros = new QCheckBox("Link left and right");
        linkEros->setChecked(true);
        erosLayout->addRow(linkEros);
        for (int s = 0; s < 2; ++s) {
            erosLayout->addRow(s == 0 ? "Left kernel (odd)" : "Right kernel (odd)",
                               sliderControl(1, 15, 7, kernel[s]));
            erosLayout->addRow(s == 0 ? "Left decay (x0.001)" : "Right decay (x0.001)",
                               sliderControl(1, 999, 300, decay[s]));
        }
        erosLayout->addRow("Raw accumulation (ms)", sliderControl(5, 200, 30, accumulation));
        columns->addWidget(erosGroup);
        outer->addWidget(settings);
        state = new QLabel("Connecting...");
        outer->addWidget(state);
        setCentralWidget(central);

        for (int s = 0; s < 2; ++s) {
            const int other = 1 - s;
            connect(sensitivity[s], &QSlider::valueChanged, this, [=](int v) {
                if (linkHardware->isChecked()) sensitivity[other]->setValue(v);
            });
            connect(refractory[s], &QSlider::valueChanged, this, [=](int v) {
                if (linkHardware->isChecked()) refractory[other]->setValue(v);
            });
            connect(kernel[s], &QSlider::valueChanged, this, [=](int v) {
                // EROS requires odd kernel sizes; reflect the actual value.
                if (v % 2 == 0) { kernel[s]->setValue(v + 1); return; }
                if (linkEros->isChecked()) kernel[other]->setValue(v);
                cameras[s].kernel = v;
                ++cameras[s].settingsVersion;
            });
            connect(decay[s], &QSlider::valueChanged, this, [=](int v) {
                if (linkEros->isChecked()) decay[other]->setValue(v);
                cameras[s].decay = v / 1000.0;
                ++cameras[s].settingsVersion;
            });
            cameras[s].start(s);
        }
        connect(accumulation, &QSlider::valueChanged, this, [this](int v) {
            for (auto &c : cameras) c.windowMs = v;
        });
        connect(apply, &QPushButton::clicked, this, [this] {
            int successful = 0;
            for (int s = 0; s < 2; ++s) {
                successful += command("sensitivity", s, sensitivity[s]->value());
                successful += command("refractory", s, refractory[s]->value());
            }
            state->setText(successful == 4
                ? "Camera biases applied successfully to both cameras."
                : "Camera RPC unavailable or rejected a setting. Check robot-side service (" +
                  QString::number(successful) + "/4 accepted).");
        });
        rpc.open("/primi-ball-balance/eventControl/rpc:o");
        auto *timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, [this] {
            QString report;
            for (int s = 0; s < 2; ++s) {
                QImage raw, eros;
                double rate;
                bool online;
                cameras[s].snapshot(raw, eros, rate, online);
                rawViews[s]->setPixmap(QPixmap::fromImage(raw).scaled(rawViews[s]->size(),
                    Qt::KeepAspectRatio, Qt::FastTransformation));
                erosViews[s]->setPixmap(QPixmap::fromImage(eros).scaled(erosViews[s]->size(),
                    Qt::KeepAspectRatio, Qt::FastTransformation));
                report += QString("%1: %2 (%3 kEv/s)  ")
                    .arg(s == 0 ? "Left" : "Right")
                    .arg(online ? "connected" : "disconnected")
                    .arg(rate / 1000.0, 0, 'f', 1);
            }
            // Keep RPC error reports visible until next user action; connection
            // status is also available in the main window's status bar.
            statusBar()->showMessage(report);
        });
        timer->start(33);
    }
    ~Window() override {
        for (auto &c : cameras) c.stop();
        rpc.close();
    }
};
} // namespace

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    yarp::os::Network yarp;
    if (!yarp::os::Network::checkNetwork(2.0)) {
        qCritical("YARP name server is unavailable");
        return 1;
    }
    Window window;
    window.show();
    return app.exec();
}
