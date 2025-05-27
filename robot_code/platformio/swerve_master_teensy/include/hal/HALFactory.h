#ifndef _HAL_FACTORY_H_
#define _HAL_FACTORY_H_

#include "HALConfig.h"
#include "ICANBus.h"
#include "IIMU.h"
#include "IRCReceiver.h"
#include "ISerial.h"
#include "IGPIO.h"
#include "IADC.h"
#include "ITimer.h"
#include "IWire.h"
#include <memory>

/**
 * @brief Factory class for creating HAL implementations
 *
 * This factory creates the appropriate HAL implementations based on
 * the compile-time HAL_IMPLEMENTATION setting.
 */
class HALFactory {
public:
    /**
     * @brief Create a CAN bus instance
     * @param busNumber CAN bus number (0, 1, etc.)
     * @return Unique pointer to CAN bus implementation
     */
    static std::unique_ptr<ICANBus> createCANBus(int busNumber = 0);

    /**
     * @brief Create an IMU instance
     * @param address I2C address for IMU (default BNO055 address)
     * @return Unique pointer to IMU implementation
     */
    static std::unique_ptr<IIMU> createIMU(uint8_t address = 0x28);

    /**
     * @brief Create an RC receiver instance
     * @param receiverType Type of receiver (SBUS, PWM, etc.)
     * @return Unique pointer to RC receiver implementation
     */
    static std::unique_ptr<IRCReceiver> createRCReceiver(const char* receiverType = "SBUS");

    /**
     * @brief Create a serial interface instance
     * @param portNumber Serial port number (0 = USB, 1 = Serial1, etc.)
     * @return Unique pointer to serial implementation
     */
    static std::unique_ptr<ISerial> createSerial(int portNumber = 0);

    /**
     * @brief Create a GPIO interface instance
     * @return Unique pointer to GPIO implementation
     */
    static std::unique_ptr<IGPIO> createGPIO();

    /**
     * @brief Create an ADC interface instance
     * @return Unique pointer to ADC implementation
     */
    static std::unique_ptr<IADC> createADC();

    /**
     * @brief Create a Timer interface instance
     * @return Unique pointer to Timer implementation
     */
    static std::unique_ptr<ITimer> createTimer();

    /**
     * @brief Create a Wire (I2C) interface instance
     * @param busNumber I2C bus number (0, 1, etc.)
     * @return Unique pointer to Wire implementation
     */
    static std::unique_ptr<IWire> createWire(int busNumber = 0);

    /**
     * @brief Get the current HAL implementation type
     * @return String describing the current implementation
     */
    static const char* getImplementationType();

    /**
     * @brief Check if running in simulation mode
     * @return true if in simulation mode
     */
    static bool isSimulation();

    /**
     * @brief Initialize the HAL system
     * This should be called once at startup
     */
    static void initialize();

    /**
     * @brief Shutdown the HAL system
     * This should be called at program exit
     */
    static void shutdown();

private:
    static bool s_initialized;
};

// Global HAL instances for easy access
namespace HAL {
    extern std::unique_ptr<ICANBus> canBus;
    extern std::unique_ptr<IIMU> imu;
    extern std::unique_ptr<IRCReceiver> rcReceiver;
    extern std::unique_ptr<ISerial> serial;
    extern std::unique_ptr<IGPIO> gpio;
    extern std::unique_ptr<IADC> adc;
    extern std::unique_ptr<ITimer> timer;
    extern std::unique_ptr<IWire> wire;

    /**
     * @brief Initialize all HAL components
     * Call this once at startup
     */
    void initialize();

    /**
     * @brief Shutdown all HAL components
     * Call this at program exit
     */
    void shutdown();
}

#endif // _HAL_FACTORY_H_
