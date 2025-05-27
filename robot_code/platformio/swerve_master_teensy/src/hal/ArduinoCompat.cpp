/**
 * @file ArduinoCompat.cpp
 * @brief Implementation of Arduino compatibility functions using HAL
 */

#include "hal/HALConfig.h"

#if HAL_IMPLEMENTATION == HAL_SIM

// Include HAL headers first to avoid macro conflicts
#include "hal/HALFactory.h"
#include "hal/IGPIO.h"
#include "hal/IADC.h"
#include "hal/ITimer.h"

// Now include Arduino compatibility (which defines conflicting macros)
#include "hal/ArduinoCompat.h"

// Undefine conflicting macros to use enum values
#undef HIGH
#undef LOW

// Global Serial instance for simulation
SerialClass Serial;

// Arduino function implementations using HAL

void pinMode(int pin, int mode) {
    if (!HAL::gpio) return;

    PinMode halMode;
    switch (mode) {
        case ARDUINO_INPUT:
            halMode = PinMode::HAL_INPUT;
            break;
        case ARDUINO_OUTPUT:
            halMode = PinMode::HAL_OUTPUT;
            break;
        case ARDUINO_INPUT_PULLUP:
            halMode = PinMode::HAL_INPUT_PULLUP;
            break;
        default:
            halMode = PinMode::HAL_INPUT;
            break;
    }

    HAL::gpio->pinMode(pin, halMode);
}

void digitalWrite(int pin, int value) {
    if (!HAL::gpio) return;

    PinState state = (value == ARDUINO_HIGH) ? PinState::HAL_HIGH : PinState::HAL_LOW;
    HAL::gpio->digitalWrite(pin, state);
}

int digitalRead(int pin) {
    if (!HAL::gpio) return ARDUINO_LOW;

    PinState state = HAL::gpio->digitalRead(pin);
    return (state == PinState::HAL_HIGH) ? ARDUINO_HIGH : ARDUINO_LOW;
}

int analogRead(int pin) {
    if (!HAL::adc) return 0;

    return HAL::adc->analogRead(pin);
}

void analogWrite(int pin, int value) {
    // PWM output - would need separate interface
    // For now, just a placeholder
}

void analogReadResolution(int bits) {
    if (!HAL::adc) return;

    HAL::adc->setResolution(bits);
}

// Note: micros() and millis() are already defined in HAL/ArduinoCompat.h as inline functions
// We don't need to redefine them here

#endif // HAL_IMPLEMENTATION == HAL_SIM
