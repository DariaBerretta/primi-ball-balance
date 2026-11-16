// Independent, opt-in I2C controller for iCub GEN3 event cameras.
// Does not reset cameras or touch the HPU acquisition device.
#include <yarp/os/Bottle.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RpcServer.h>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <initializer_list>
#include <iostream>
#include <string>

namespace {
constexpr const char* device = "/dev/i2c-0";
constexpr int addresses[2] = {0x10, 0x11};
constexpr uint8_t infoReg = 0x00;
constexpr uint8_t sisAddressReg = 0x24;
constexpr uint8_t sisDataReg = 0x28;

// The camera-controller protocol is the one used by event-driven's
// zynqGrabber/vsctrlDevice.h: four-byte transfers use auto-increment.
bool readRegister(int fd, uint8_t reg, uint32_t& result) {
    const uint8_t address = reg | 0x80;
    return ::write(fd, &address, 1) == 1 &&
           ::read(fd, &result, sizeof(result)) == sizeof(result);
}

bool writeRegister(int fd, uint8_t reg, uint32_t value) {
    uint8_t bytes[5];
    bytes[0] = reg | 0x80;
    std::memcpy(bytes + 1, &value, sizeof(value));
    return ::write(fd, bytes, sizeof(bytes)) == sizeof(bytes);
}

bool readBiasRegister(int fd, uint32_t address, uint32_t& result) {
    // Select a Sisley register for reading (high bit clear).
    return writeRegister(fd, sisAddressReg, address & 0x00ffffff) &&
           readRegister(fd, sisDataReg, result);
}

bool writeBiasRegister(int fd, uint32_t address, uint32_t value) {
    // The high bit of the Sisley address commits the previously written data.
    return writeRegister(fd, sisDataReg, value) &&
           writeRegister(fd, sisAddressReg, (address & 0x00ffffff) | 0x80000000);
}

bool selectCamera(int fd, int side) {
    return ::ioctl(fd, I2C_SLAVE, addresses[side]) == 0;
}

bool identify(int fd, int side, uint32_t& info) {
    return selectCamera(fd, side) && readRegister(fd, infoReg, info) &&
           ((info >> 13) & 7) == 3;
}

// Read current bias-register values without changing the bias settings.
bool inspectCamera(int fd, int side, std::ostream& output) {
    uint32_t info = 0;
    if (!identify(fd, side, info)) return false;
    output << (side == 0 ? "left" : "right") << ": GEN3 v"
           << ((info >> 4) & 15) << '.' << (info & 15) << '\n';
    for (uint32_t reg : {0x128u, 0x14cu, 0x150u, 0x154u}) {
        uint32_t value = 0;
        if (!readBiasRegister(fd, reg, value)) return false;
        output << "  Sisley 0x" << std::hex << reg << " = 0x"
               << std::setw(8) << std::setfill('0') << value
               << std::dec << std::setfill(' ') << '\n';
    }
    return true;
}

// Bias formulas and register addresses are from the installed
// event-driven zynqGrabber/vsctrlDevice.h, not a new calibration.
// Exact register readback confirms the write, not physical sensor calibration.
bool changeBias(int fd, int side, const std::string& setting, int value) {
    uint32_t info = 0;
    if (!identify(fd, side, info)) return false;

    auto update = [fd](uint32_t address, uint32_t bits) {
        uint32_t oldValue = 0, observed = 0;
        if (!readBiasRegister(fd, address, oldValue)) return false;
        const uint32_t requested = (oldValue & 0xff300000u) + bits;
        if (!writeBiasRegister(fd, address, requested)) return false;
        return readBiasRegister(fd, address, observed) && observed == requested;
    };

    if (setting == "sensitivity") {
        if (value < 10 || value > 90) return false;
        const double s = 10 + 13 * (1.0 - value * 0.01);
        // Casts intentionally match zynqGrabber's truncation to uint32_t.
        return update(0x154, 43) &&
               update(0x14c, static_cast<uint32_t>(43 - s)) &&
               update(0x150, static_cast<uint32_t>(43 + s));
    }
    if (setting == "refractory") {
        if (value < 1 || value > 99) return false;
        return update(0x128, static_cast<uint32_t>(245 - 0.5 * (100 - value)));
    }
    return false;
}

class CameraControl : public yarp::os::RFModule {
    yarp::os::RpcServer rpc;
    bool allowWrites = false;

public:
    bool configure(yarp::os::ResourceFinder& rf) override {
        allowWrites = rf.check("allow-writes");
        const std::string name = "/primi-ball-balance/cameraControl/rpc";
        if (!rpc.open(name)) return false;
        if (!attach(rpc)) return false;
        std::cout << "Camera control listening on " << name << '\n'
                  << (allowWrites ? "Register writes ENABLED" : "READ ONLY") << '\n';
        return true;
    }

    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply) override {
        // Supported commands:
        //   inspect left|right
        //   set sensitivity|refractory left|right <value>
        const std::string verb = command.size() ? command.get(0).asString() : "";
        const bool settingCommand = verb == "set" && command.size() == 4;
        const bool inspectCommand = verb == "inspect" && command.size() == 2;
        if (!settingCommand && !inspectCommand) {
            reply.addString("error");
            reply.addString("expected inspect <side> or set <setting> <side> <value>");
            return true;
        }
        const std::string sideName = command.get(settingCommand ? 2 : 1).asString();
        if (sideName != "left" && sideName != "right") {
            reply.addString("error");
            reply.addString("side must be left or right");
            return true;
        }
        const int side = sideName == "left" ? 0 : 1;
        if (settingCommand && !allowWrites) {
            reply.addString("error");
            reply.addString("server is read-only; restart with --allow-writes");
            return true;
        }
        const std::string setting = settingCommand ? command.get(1).asString() : "";
        const int value = settingCommand ? command.get(3).asInt32() : 0;
        if (settingCommand &&
            !((setting == "sensitivity" && value >= 10 && value <= 90) ||
              (setting == "refractory" && value >= 1 && value <= 99))) {
            reply.addString("error");
            reply.addString("invalid setting or value");
            return true;
        }

        const int fd = ::open(device, O_RDWR);
        if (fd < 0) {
            reply.addString("error");
            reply.addString(std::strerror(errno));
            return true;
        }
        bool ok = false;
        if (settingCommand) {
            ok = changeBias(fd, side, setting, value);
            std::cout << sideName << ' ' << setting << ' ' << value
                      << (ok ? " readback OK" : " FAILED") << '\n';
        } else {
            ok = inspectCamera(fd, side, std::cout);
        }
        ::close(fd);
        reply.addString(ok ? "ok" : "error");
        if (!ok) reply.addString("I2C communication or register readback failed");
        return true;
    }

    double getPeriod() override { return 1.0; }
    bool updateModule() override { return true; }
    bool interruptModule() override { rpc.interrupt(); return true; }
    bool close() override { rpc.close(); return true; }
};
} // namespace

int main(int argc, char** argv) {
    if (argc == 1 || (argc == 2 && std::string(argv[1]) == "--inspect")) {
        const int fd = ::open(device, O_RDWR);
        if (fd < 0) {
            std::cerr << "Cannot open " << device << ": " << std::strerror(errno) << '\n';
            return 1;
        }
        bool ok = inspectCamera(fd, 0, std::cout) && inspectCamera(fd, 1, std::cout);
        ::close(fd);
        return ok ? 0 : 1;
    }
    [[maybe_unused]] yarp::os::Network network;
    if (!yarp::os::Network::checkNetwork(2.0)) {
        std::cerr << "YARP name server unavailable\n";
        return 1;
    }
    if (std::string(argv[1]) != "--serve" || argc > 3 ||
        (argc == 3 && std::string(argv[2]) != "--allow-writes")) {
        std::cerr << "Usage: camera_control [--inspect] | --serve [--allow-writes]\n";
        return 2;
    }
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    CameraControl service;
    return service.runModule(rf);
}
