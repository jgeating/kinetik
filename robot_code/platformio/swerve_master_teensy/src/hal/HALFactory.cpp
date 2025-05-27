#include "hal/HALFactory.h"

#if HAL_IMPLEMENTATION == HAL_SIM
#include <iostream>
#include <iomanip>
#include <cstring>
#include <chrono>
#include <unordered_map>
#include <vector>
#include <thread>

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

class MockGPIO : public IGPIO {
private:
    std::unordered_map<uint8_t, PinMode> pinModes;
    std::unordered_map<uint8_t, PinState> pinStates;

public:
    void pinMode(uint8_t pin, PinMode mode) override {
        pinModes[pin] = mode;
        std::cout << "GPIO: Set pin " << (int)pin << " to mode " << (int)mode << std::endl;
    }

    void digitalWrite(uint8_t pin, PinState state) override {
        pinStates[pin] = state;
        std::cout << "GPIO: Set pin " << (int)pin << " to " << (state == PinState::HAL_HIGH ? "HIGH" : "LOW") << std::endl;
    }

    PinState digitalRead(uint8_t pin) override {
        auto it = pinStates.find(pin);
        if (it != pinStates.end()) {
            return it->second;
        }
        return PinState::HAL_LOW; // Default to LOW
    }

    bool begin() override { return true; }
    bool isReady() const override { return true; }
};

class MockADC : public IADC {
private:
    uint8_t resolution = 12;

public:
    uint16_t analogRead(uint8_t pin) override {
        // Return a simulated value based on pin number
        uint16_t maxVal = getMaxValue();
        return (pin * 100) % maxVal; // Simple simulation
    }

    void setResolution(uint8_t bits) override {
        resolution = bits;
        std::cout << "ADC: Set resolution to " << (int)bits << " bits" << std::endl;
    }

    uint8_t getResolution() const override { return resolution; }

    uint16_t getMaxValue() const override {
        return (1 << resolution) - 1;
    }

    bool begin() override { return true; }
    bool isReady() const override { return true; }

    float toVoltage(uint16_t reading, float referenceVoltage) const override {
        return (float)reading * referenceVoltage / getMaxValue();
    }
};

class MockTimer : public ITimer {
private:
    std::chrono::steady_clock::time_point startTime;

public:
    MockTimer() : startTime(std::chrono::steady_clock::now()) {}

    uint32_t micros() override {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - startTime);
        return static_cast<uint32_t>(duration.count());
    }

    uint32_t millis() override {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime);
        return static_cast<uint32_t>(duration.count());
    }

    void delay(uint32_t ms) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }

    void delayMicroseconds(uint32_t us) override {
        std::this_thread::sleep_for(std::chrono::microseconds(us));
    }

    bool begin() override { return true; }
    bool isReady() const override { return true; }

    uint64_t getHighResTime() override {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - startTime);
        return static_cast<uint64_t>(duration.count());
    }

    void reset() override {
        startTime = std::chrono::steady_clock::now();
    }
};

class MockWire : public IWire {
private:
    uint8_t currentAddress = 0;
    std::vector<uint8_t> txBuffer;
    std::vector<uint8_t> rxBuffer;

public:
    bool begin() override { return true; }
    bool begin(uint8_t address) override { return true; }
    void setClock(uint32_t frequency) override {}

    void beginTransmission(uint8_t address) override {
        currentAddress = address;
        txBuffer.clear();
    }

    uint8_t endTransmission(bool stop) override {
        std::cout << "I2C: Sent " << txBuffer.size() << " bytes to address 0x"
                  << std::hex << (int)currentAddress << std::dec << std::endl;
        txBuffer.clear();
        return 0; // Success
    }

    size_t write(uint8_t data) override {
        txBuffer.push_back(data);
        return 1;
    }

    size_t write(const uint8_t* data, size_t length) override {
        for (size_t i = 0; i < length; i++) {
            txBuffer.push_back(data[i]);
        }
        return length;
    }

    uint8_t requestFrom(uint8_t address, uint8_t quantity, bool stop) override {
        // Simulate receiving data
        rxBuffer.clear();
        for (uint8_t i = 0; i < quantity; i++) {
            rxBuffer.push_back(i); // Simple test data
        }
        return quantity;
    }

    int available() override { return rxBuffer.size(); }

    int read() override {
        if (rxBuffer.empty()) return -1;
        uint8_t data = rxBuffer.front();
        rxBuffer.erase(rxBuffer.begin());
        return data;
    }

    int peek() override {
        if (rxBuffer.empty()) return -1;
        return rxBuffer.front();
    }

    bool isReady() const override { return true; }
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
    std::unique_ptr<IGPIO> gpio;
    std::unique_ptr<IADC> adc;
    std::unique_ptr<ITimer> timer;
    std::unique_ptr<IWire> wire;

    void initialize() {
        HALFactory::initialize();

        canBus = HALFactory::createCANBus();
        imu = HALFactory::createIMU();
        rcReceiver = HALFactory::createRCReceiver();
        serial = HALFactory::createSerial();
        gpio = HALFactory::createGPIO();
        adc = HALFactory::createADC();
        timer = HALFactory::createTimer();
        wire = HALFactory::createWire();
    }

    void shutdown() {
        canBus.reset();
        imu.reset();
        rcReceiver.reset();
        serial.reset();
        gpio.reset();
        adc.reset();
        timer.reset();
        wire.reset();

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

std::unique_ptr<IGPIO> HALFactory::createGPIO() {
#if HAL_IMPLEMENTATION == HAL_REAL
    // TODO: Implement real GPIO
    return nullptr;
#elif HAL_IMPLEMENTATION == HAL_SIM
    return std::make_unique<MockGPIO>();
#endif
}

std::unique_ptr<IADC> HALFactory::createADC() {
#if HAL_IMPLEMENTATION == HAL_REAL
    // TODO: Implement real ADC
    return nullptr;
#elif HAL_IMPLEMENTATION == HAL_SIM
    return std::make_unique<MockADC>();
#endif
}

std::unique_ptr<ITimer> HALFactory::createTimer() {
#if HAL_IMPLEMENTATION == HAL_REAL
    // TODO: Implement real Timer
    return nullptr;
#elif HAL_IMPLEMENTATION == HAL_SIM
    return std::make_unique<MockTimer>();
#endif
}

std::unique_ptr<IWire> HALFactory::createWire(int busNumber) {
#if HAL_IMPLEMENTATION == HAL_REAL
    // TODO: Implement real Wire
    return nullptr;
#elif HAL_IMPLEMENTATION == HAL_SIM
    return std::make_unique<MockWire>();
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
