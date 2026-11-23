// BMA400 USB reader and YARP publisher for primi-ball-balance.
//
// One independent Linux process owns the USB serial device. A single
// acquisition/publishing loop is sufficient at 100 Hz and avoids unnecessary
// queueing delays. No robot control API is opened or used by this program.
//
// YARP Bottle layout (five numeric fields, matching the previous Python reader):
//   [feather_timestamp_us, ax_m_s2, ay_m_s2, az_m_s2, pc_receive_unix_s]
// The YARP Stamp envelope contains an incremental sequence number and the
// PC reception timestamp. Both clocks are retained but NOT synchronized.

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Stamp.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t stopRequested = 0;

void requestStop(int) { stopRequested = 1; }

struct Config {
    std::string serialPort = "/dev/ttyACM0";
    int baud = 115200;
    std::string yarpPort = "/primi-ball-balance/imu/bma400:o";
    std::string csvPath;
    double timeoutSeconds = 1.0;
    double maxSilenceSeconds = 10.0;
};

std::string envOr(const char* name, const std::string& fallback) {
    const char* value = std::getenv(name);
    return value != nullptr ? std::string(value) : fallback;
}

int parsePositiveInt(const std::string& text, const char* name) {
    std::size_t end = 0;
    const long value = std::stol(text, &end);
    if (end != text.size() || value <= 0 || value > std::numeric_limits<int>::max()) {
        throw std::runtime_error(std::string("Invalid ") + name + ": " + text);
    }
    return static_cast<int>(value);
}

double parsePositiveDouble(const std::string& text, const char* name) {
    std::size_t end = 0;
    const double value = std::stod(text, &end);
    if (end != text.size() || !std::isfinite(value) || value <= 0.0) {
        throw std::runtime_error(std::string("Invalid ") + name + ": " + text);
    }
    return value;
}

Config parseConfig(int argc, char* argv[]) {
    Config cfg;
    cfg.serialPort = envOr("BMA400_SERIAL_PORT", cfg.serialPort);
    cfg.baud = parsePositiveInt(envOr("BMA400_BAUD", "115200"), "baud");
    cfg.yarpPort = envOr("BMA400_YARP_PORT", cfg.yarpPort);
    cfg.csvPath = envOr("BMA400_CSV", "");
    cfg.timeoutSeconds = parsePositiveDouble(envOr("BMA400_READ_TIMEOUT", "1.0"), "timeout");
    cfg.maxSilenceSeconds = parsePositiveDouble(envOr("BMA400_MAX_SILENCE", "10.0"), "max silence");

    // Preserve the previous Python reader's command-line interface.
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: bma400_reader [--port DEVICE] [--baud RATE] "
                         "[--yarp-port NAME] [--csv PATH] "
                         "[--timeout SECONDS] [--max-silence SECONDS]\n";
            std::exit(0);
        }
        if (i + 1 >= argc) {
            throw std::runtime_error("Missing value after " + arg);
        }
        const std::string value(argv[++i]);
        if (arg == "--port") cfg.serialPort = value;
        else if (arg == "--baud") cfg.baud = parsePositiveInt(value, "baud");
        else if (arg == "--yarp-port") cfg.yarpPort = value;
        else if (arg == "--csv") cfg.csvPath = value;
        else if (arg == "--timeout") cfg.timeoutSeconds = parsePositiveDouble(value, "timeout");
        else if (arg == "--max-silence") cfg.maxSilenceSeconds = parsePositiveDouble(value, "max silence");
        else throw std::runtime_error("Unknown option: " + arg);
    }
    if (cfg.serialPort.empty() || cfg.yarpPort.empty() || cfg.yarpPort[0] != '/') {
        throw std::runtime_error("Invalid serial device or YARP output port");
    }
    return cfg;
}

speed_t baudToTermios(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
#ifdef B230400
        case 230400: return B230400;
#endif
        default: throw std::runtime_error("Unsupported serial baud rate: " + std::to_string(baud));
    }
}

class SerialDevice {
public:
    SerialDevice(const std::string& path, int baud) {
        fd_ = ::open(path.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) throw std::runtime_error("Cannot open " + path + ": " + std::strerror(errno));
        try {
            struct termios settings {};
            if (::tcgetattr(fd_, &settings) != 0) {
                throw std::runtime_error("tcgetattr failed: " + std::string(std::strerror(errno)));
            }
            ::cfmakeraw(&settings);
            const speed_t speed = baudToTermios(baud);
            if (::cfsetispeed(&settings, speed) != 0 || ::cfsetospeed(&settings, speed) != 0) {
                throw std::runtime_error("Cannot set serial baud rate");
            }
            settings.c_cflag |= (CLOCAL | CREAD);
            settings.c_cflag &= ~CRTSCTS;
            settings.c_cflag &= ~CSIZE;
            settings.c_cflag |= CS8;
            settings.c_cflag &= ~(PARENB | CSTOPB);
            settings.c_cc[VMIN] = 0;
            settings.c_cc[VTIME] = 0;
            if (::tcsetattr(fd_, TCSANOW, &settings) != 0) {
                throw std::runtime_error("tcsetattr failed: " + std::string(std::strerror(errno)));
            }
            // Discard incomplete/status data buffered before the reader started.
            if (::tcflush(fd_, TCIFLUSH) != 0) {
                throw std::runtime_error("tcflush failed: " + std::string(std::strerror(errno)));
            }
        } catch (...) {
            ::close(fd_);
            fd_ = -1;
            throw;
        }
    }

    SerialDevice(const SerialDevice&) = delete;
    SerialDevice& operator=(const SerialDevice&) = delete;
    ~SerialDevice() { if (fd_ >= 0) ::close(fd_); }
    int fd() const { return fd_; }

private:
    int fd_ = -1;
};

struct Sample {
    std::uint32_t featherUs;
    double ax;
    double ay;
    double az;
};

bool parseSample(const std::string& line, Sample& sample) {
    // The firmware sends exactly four comma-separated numeric fields.
    // Startup messages and malformed/incomplete rows are ignored.
    std::string fields[4];
    std::size_t start = 0;
    for (int i = 0; i < 4; ++i) {
        const std::size_t pos = line.find(',', start);
        if ((i < 3 && pos == std::string::npos) ||
            (i == 3 && pos != std::string::npos)) return false;
        fields[i] = line.substr(start, pos == std::string::npos ? pos : pos - start);
        if (fields[i].empty()) return false;
        start = pos == std::string::npos ? line.size() : pos + 1;
    }
    try {
        if (fields[0][0] == '-') return false;
        std::size_t consumed = 0;
        const unsigned long long raw = std::stoull(fields[0], &consumed);
        if (consumed != fields[0].size() || raw > UINT32_MAX) return false;
        sample.featherUs = static_cast<std::uint32_t>(raw);
        double* output[3] = {&sample.ax, &sample.ay, &sample.az};
        for (int i = 0; i < 3; ++i) {
            consumed = 0;
            *output[i] = std::stod(fields[i + 1], &consumed);
            if (consumed != fields[i + 1].size() || !std::isfinite(*output[i])) return false;
        }
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

double unixNowSeconds() {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration<double>(now).count();
}

void publish(yarp::os::BufferedPort<yarp::os::Bottle>& port,
             const Sample& sample, double pcReceiveUnixSeconds,
             std::uint64_t sequence) {
    yarp::os::Bottle& bottle = port.prepare();
    bottle.clear();
    bottle.addFloat64(static_cast<double>(sample.featherUs));
    bottle.addFloat64(sample.ax);
    bottle.addFloat64(sample.ay);
    bottle.addFloat64(sample.az);
    bottle.addFloat64(pcReceiveUnixSeconds);

    // The envelope is in the PC Unix-time domain, not the Feather clock.
    // As with the Python version, this is a receipt timestamp, not a
    // hardware-acquisition timestamp. Synchronization is a later step.
    yarp::os::Stamp stamp(static_cast<int>(sequence % static_cast<std::uint64_t>(INT32_MAX)),
                          pcReceiveUnixSeconds);
    port.setEnvelope(stamp);
    port.write();
}

int run(const Config& cfg) {
    yarp::os::Network yarpNetwork;
    (void)yarpNetwork;
    if (!yarp::os::Network::checkNetwork(2.0)) {
        throw std::runtime_error("YARP name server is not reachable");
    }

    yarp::os::BufferedPort<yarp::os::Bottle> output;
    if (!output.open(cfg.yarpPort)) {
        throw std::runtime_error("Cannot open YARP port: " + cfg.yarpPort);
    }

    SerialDevice serial(cfg.serialPort, cfg.baud);

    std::ofstream csv;
    if (!cfg.csvPath.empty()) {
        // The launcher runs from the repository root. Create the parent
        // directory (if needed) in the launcher, not in the control loop.
        csv.open(cfg.csvPath, std::ios::out | std::ios::trunc);
        if (!csv) throw std::runtime_error("Cannot open CSV file: " + cfg.csvPath);
        csv << "feather_timestamp_us,ax_m_s2,ay_m_s2,az_m_s2,pc_receive_unix_s,yarp_receive_time_s\n";
        csv << std::setprecision(17);
        std::cout << "CSV recording: " << cfg.csvPath << '\n';
    } else {
        std::cout << "CSV recording: disabled\n";
    }

    std::cout << "USB source: " << cfg.serialPort << '\n'
              << "YARP output: " << cfg.yarpPort << '\n'
              << "Bottle fields: feather_us ax ay az pc_receive_unix_s\n"
              << "Waiting for the first valid BMA400 sample..." << std::endl;

    using Steady = std::chrono::steady_clock;
    auto lastValid = Steady::now();
    auto lastReport = lastValid;
    std::uint64_t published = 0;
    std::uint64_t ignored = 0;
    std::string pending;

    while (!stopRequested) {
        struct pollfd descriptor {serial.fd(), static_cast<short>(POLLIN | POLLERR | POLLHUP), 0};
        // A short timeout allows a clean SIGINT/SIGTERM shutdown and periodic
        // detection of sensor silence without blocking the robot controller.
        const int waitMillis = static_cast<int>(std::max(1.0, std::min(cfg.timeoutSeconds * 1000.0, 200.0)));
        const int status = ::poll(&descriptor, 1, waitMillis);
        if (status < 0) {
            if (errno == EINTR) continue;
            throw std::runtime_error("Serial poll failed: " + std::string(std::strerror(errno)));
        }
        if (status > 0 && (descriptor.revents & POLLIN)) {
            char chunk[512];
            const ssize_t bytes = ::read(serial.fd(), chunk, sizeof(chunk));
            if (bytes > 0) {
                for (ssize_t i = 0; i < bytes; ++i) {
                    const char ch = chunk[i];
                    if (ch == '\n') {
                        if (!pending.empty() && pending.back() == '\r') pending.pop_back();
                        const double receptionTime = unixNowSeconds();
                        Sample sample {};
                        if (parseSample(pending, sample)) {
                            publish(output, sample, receptionTime, published);
                            if (csv.is_open()) {
                                csv << sample.featherUs << ',' << sample.ax << ','
                                    << sample.ay << ',' << sample.az << ','
                                    << receptionTime << ',' << receptionTime << '\n';
                                if (!csv) throw std::runtime_error("Failed to write CSV data");
                            }
                            ++published;
                            lastValid = Steady::now();
                        } else {
                            ++ignored;
                        }
                        pending.clear();
                    } else if (pending.size() < 512) {
                        pending.push_back(ch);
                    } else {
                        // Oversized/incomplete lines cannot be valid firmware CSV.
                        pending.clear();
                        ++ignored;
                    }
                }
            } else if (bytes == 0) {
                throw std::runtime_error("USB serial device disconnected");
            } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                throw std::runtime_error("Serial read failed: " + std::string(std::strerror(errno)));
            }
        }
        if (status > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))) {
            throw std::runtime_error("USB serial device disconnected or failed");
        }
        const auto now = Steady::now();
        if (std::chrono::duration<double>(now - lastReport).count() >= 5.0) {
            std::cout << "Published: " << published << " | ignored lines: " << ignored << std::endl;
            if (csv.is_open()) csv.flush();
            lastReport = now;
        }
        if (std::chrono::duration<double>(now - lastValid).count() > cfg.maxSilenceSeconds) {
            throw std::runtime_error("No valid BMA400 samples for " +
                                     std::to_string(cfg.maxSilenceSeconds) + " seconds");
        }
    }
    if (csv.is_open()) csv.flush();
    std::cout << "Stopped. Published " << published << " samples." << std::endl;
    output.close();
    return 0;
}
} // namespace

int main(int argc, char* argv[]) {
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);
    try {
        return run(parseConfig(argc, argv));
    } catch (const std::exception& error) {
        std::cerr << "ERROR: " << error.what() << std::endl;
        return 1;
    }
}
