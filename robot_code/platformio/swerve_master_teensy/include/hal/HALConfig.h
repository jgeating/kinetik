#ifndef _HAL_CONFIG_H_
#define _HAL_CONFIG_H_

// HAL Implementation Selection
#define HAL_REAL 1
#define HAL_SIM  2

// Default to real hardware if not specified
#ifndef HAL_IMPLEMENTATION
#define HAL_IMPLEMENTATION HAL_REAL
#endif

// Platform-specific includes
#if HAL_IMPLEMENTATION == HAL_REAL
    // Real hardware includes (Teensy/Arduino)
    #include <Arduino.h>
    #include <FlexCAN_T4.h>
    #include <Wire.h>
    #include "Adafruit_BNO055.h"
    #include "Adafruit_Sensor.h"
#elif HAL_IMPLEMENTATION == HAL_SIM
    // Simulation includes (Native C++)
    #include <iostream>
    #include <chrono>
    #include <string>
    #include <vector>
    #include <memory>
    #include <cstring>
    // Note: Threading and JSON will be included when needed

    // Mock Arduino types for compatibility
    typedef uint8_t byte;
    typedef bool boolean;

    // Mock timing functions
    inline unsigned long millis() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    inline unsigned long micros() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    // Note: delay functions will be implemented in ArduinoCompat.h
    // to avoid threading dependencies in this header
#endif

#endif // _HAL_CONFIG_H_
