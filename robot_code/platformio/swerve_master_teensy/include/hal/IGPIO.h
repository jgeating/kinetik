#ifndef _IGPIO_H_
#define _IGPIO_H_

#include <stdint.h>

/**
 * @brief GPIO pin modes
 */
enum class PinMode {
    HAL_INPUT = 0,
    HAL_OUTPUT = 1,
    HAL_INPUT_PULLUP = 2,
    HAL_INPUT_PULLDOWN = 3
};

/**
 * @brief GPIO pin states
 */
enum class PinState {
    HAL_LOW = 0,
    HAL_HIGH = 1
};

/**
 * @brief Interface for GPIO (General Purpose Input/Output) operations
 *
 * This interface provides platform-independent access to digital I/O pins.
 * It abstracts Arduino functions like pinMode(), digitalWrite(), and digitalRead().
 */
class IGPIO {
public:
    virtual ~IGPIO() = default;

    /**
     * @brief Configure a pin's mode
     * @param pin Pin number
     * @param mode Pin mode (INPUT, OUTPUT, INPUT_PULLUP, INPUT_PULLDOWN)
     */
    virtual void pinMode(uint8_t pin, PinMode mode) = 0;

    /**
     * @brief Write a digital value to a pin
     * @param pin Pin number
     * @param state Pin state (LOW or HIGH)
     */
    virtual void digitalWrite(uint8_t pin, PinState state) = 0;

    /**
     * @brief Read a digital value from a pin
     * @param pin Pin number
     * @return Pin state (LOW or HIGH)
     */
    virtual PinState digitalRead(uint8_t pin) = 0;

    /**
     * @brief Initialize the GPIO system
     * @return true if initialization successful
     */
    virtual bool begin() = 0;

    /**
     * @brief Check if GPIO system is ready
     * @return true if ready for operations
     */
    virtual bool isReady() const = 0;
};

#endif // _IGPIO_H_
