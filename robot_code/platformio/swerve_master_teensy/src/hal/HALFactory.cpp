#include "hal/HALFactory.h"

#if HAL_IMPLEMENTATION == HAL_SIM
#include <iostream>
#include <iomanip>
#include <cstring>

// For simulation, create simple mock implementations
class MockCANBus : public ICANBus {
public:
    bool begin(uint32_t baudRate = 1000000) override { return true; }
    bool write(const CANMessage& msg) override { return true; }
    bool available() override { return false; }
    bool read(CANMessage& msg) override { return false; }
    void setBaudRate(uint32_t baudRate) override {}
    uint32_t getBaudRate() const override { return 1000000; }
    void setDebugMode(bool enable) override {}
};

class MockIMU : public IIMU {
public:
    bool begin() override { return true; }
    Vector3D getEulerAngles() override { return Vector3D(0, 0, 0); }
    Vector3D getAngularVelocity() override { return Vector3D(0, 0, 0); }
    Vector3D getLinearAcceleration() override { return Vector3D(0, 0, 9.81); }
    Vector3D getMagnetometer() override { return Vector3D(0, 0, 0); }
    Quaternion getQuaternion() override { return Quaternion(1, 0, 0, 0); }
    int8_t getTemperature() override { return 25; }
    IMUCalibration getCalibrationStatus() override { return IMUCalibration(); }
    bool isConnected() override { return true; }
    void setExternalCrystal(bool useExternal) override {}
};

class MockRCReceiver : public IRCReceiver {
public:
    bool begin() override { return true; }
    void update() override {}
    double getChannelData(RCChannel channel, double defaultValue = 0.0) override { return defaultValue; }
    int getRawChannelData(RCChannel channel) override { return 1500; }
    bool isSignalLost() override { return false; }
    unsigned long getTimeSinceLastSignal() override { return 0; }
    RCData getAllData() override { return RCData(); }

    double getLeftVertical() override { return 0.0; }
    double getLeftHorizontal() override { return 0.0; }
    double getRightVertical() override { return 0.0; }
    double getRightHorizontal() override { return 0.0; }
    double getLeftKnob() override { return 0.0; }
    double getRightKnob() override { return 0.0; }
    int getBlueSwitch() override { return 0; }
    int getRedSwitch() override { return 1; }
    bool isBlueSwitchUp() override { return false; }
    bool isBlueSwitchDown() override { return false; }
    bool isBlueSwitchCentered() override { return true; }
};

class MockSerial : public ISerial {
public:
    void begin(unsigned long baudRate) override {}
    void end() override {}
    bool isReady() override { return true; }
    int available() override { return 0; }
    int read() override { return -1; }
    size_t readBytes(uint8_t* buffer, size_t length) override { return 0; }
    size_t write(uint8_t data) override { return 1; }
    size_t write(const uint8_t* buffer, size_t length) override { return length; }
    size_t write(const char* str) override { return strlen(str); }
    size_t print(const char* str) override {
#if HAL_IMPLEMENTATION == HAL_SIM
        std::cout << str;
#endif
        return strlen(str);
    }
    size_t println(const char* str) override {
#if HAL_IMPLEMENTATION == HAL_SIM
        std::cout << str << std::endl;
#endif
        return strlen(str) + 1;
    }
    size_t print(int value, int base = 10) override {
#if HAL_IMPLEMENTATION == HAL_SIM
        std::cout << value;
#endif
        return 1;
    }
    size_t print(double value, int digits = 2) override {
#if HAL_IMPLEMENTATION == HAL_SIM
        std::cout << std::fixed << std::setprecision(digits) << value;
#endif
        return 1;
    }
    size_t println(int value, int base = 10) override {
#if HAL_IMPLEMENTATION == HAL_SIM
        std::cout << value << std::endl;
#endif
        return 1;
    }
    size_t println(double value, int digits = 2) override {
#if HAL_IMPLEMENTATION == HAL_SIM
        std::cout << std::fixed << std::setprecision(digits) << value << std::endl;
#endif
        return 1;
    }
    void flush() override {}
    void setTimeout(unsigned long timeout) override {}
};

#endif

// Static member initialization
bool HALFactory::s_initialized = false;

// Global HAL instances
namespace HAL {
    std::unique_ptr<ICANBus> canBus;
    std::unique_ptr<IIMU> imu;
    std::unique_ptr<IRCReceiver> rcReceiver;
    std::unique_ptr<ISerial> serial;

    void initialize() {
        HALFactory::initialize();

        canBus = HALFactory::createCANBus();
        imu = HALFactory::createIMU();
        rcReceiver = HALFactory::createRCReceiver();
        serial = HALFactory::createSerial();
    }

    void shutdown() {
        canBus.reset();
        imu.reset();
        rcReceiver.reset();
        serial.reset();

        HALFactory::shutdown();
    }
}

// HALFactory implementation
std::unique_ptr<ICANBus> HALFactory::createCANBus(int busNumber) {
#if HAL_IMPLEMENTATION == HAL_REAL
    // TODO: Implement real CAN bus
    return nullptr;
#elif HAL_IMPLEMENTATION == HAL_SIM
    return std::make_unique<MockCANBus>();
#endif
}

std::unique_ptr<IIMU> HALFactory::createIMU(uint8_t address) {
#if HAL_IMPLEMENTATION == HAL_REAL
    // TODO: Implement real IMU
    return nullptr;
#elif HAL_IMPLEMENTATION == HAL_SIM
    return std::make_unique<MockIMU>();
#endif
}

std::unique_ptr<IRCReceiver> HALFactory::createRCReceiver(const char* receiverType) {
#if HAL_IMPLEMENTATION == HAL_REAL
    // TODO: Implement real RC receiver
    return nullptr;
#elif HAL_IMPLEMENTATION == HAL_SIM
    return std::make_unique<MockRCReceiver>();
#endif
}

std::unique_ptr<ISerial> HALFactory::createSerial(int portNumber) {
#if HAL_IMPLEMENTATION == HAL_REAL
    // TODO: Implement real serial
    return nullptr;
#elif HAL_IMPLEMENTATION == HAL_SIM
    return std::make_unique<MockSerial>();
#endif
}

const char* HALFactory::getImplementationType() {
#if HAL_IMPLEMENTATION == HAL_REAL
    return "Real Hardware";
#elif HAL_IMPLEMENTATION == HAL_SIM
    return "Simulation";
#else
    return "Unknown";
#endif
}

bool HALFactory::isSimulation() {
#if HAL_IMPLEMENTATION == HAL_SIM
    return true;
#else
    return false;
#endif
}

void HALFactory::initialize() {
    if (s_initialized) return;

    // Platform-specific initialization
#if HAL_IMPLEMENTATION == HAL_SIM
    // Simulation initialization
    std::cout << "Initializing HAL in simulation mode..." << std::endl;
#elif HAL_IMPLEMENTATION == HAL_REAL
    // Real hardware initialization
    Serial.println("Initializing HAL in real hardware mode...");
#endif

    s_initialized = true;
}

void HALFactory::shutdown() {
    if (!s_initialized) return;

    // Platform-specific cleanup
#if HAL_IMPLEMENTATION == HAL_SIM
    std::cout << "Shutting down HAL..." << std::endl;
#elif HAL_IMPLEMENTATION == HAL_REAL
    Serial.println("Shutting down HAL...");
#endif

    s_initialized = false;
}
