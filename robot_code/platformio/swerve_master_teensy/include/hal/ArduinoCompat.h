#ifndef _ARDUINO_COMPAT_H_
#define _ARDUINO_COMPAT_H_

#include "HALConfig.h"

#if HAL_IMPLEMENTATION == HAL_SIM

/**
 * @brief Arduino compatibility layer for simulation
 *
 * This header provides Arduino-compatible functions and classes
 * that work in the native simulation environment.
 */

#include <cstdint>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <thread>
#include <chrono>

// Arduino-style type definitions
typedef uint8_t byte;
typedef bool boolean;

// Arduino constants for simulation
const int ARDUINO_HIGH = 1;
const int ARDUINO_LOW = 0;
const int ARDUINO_INPUT = 0;
const int ARDUINO_OUTPUT = 1;
const int ARDUINO_INPUT_PULLUP = 2;

// Define Arduino macros for compatibility (only in simulation)
// Note: These may conflict with HAL enums, so be careful with includes
#define HIGH ARDUINO_HIGH
#define LOW ARDUINO_LOW

// Math constants
#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

// Arduino-style functions
inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline long constrain(long x, long a, long b) {
    return std::max(a, std::min(x, b));
}

inline double constrainf(double x, double a, double b) {
    return std::max(a, std::min(x, b));
}

inline double radians(double degrees) {
    return degrees * PI / 180.0;
}

inline double degrees(double radians) {
    return radians * 180.0 / PI;
}

inline double sq(double x) {
    return x * x;
}

inline double sqrt(double x) {
    return std::sqrt(x);
}

// abs() functions removed to avoid conflicts with std::abs

inline double sin(double x) {
    return std::sin(x);
}

inline double cos(double x) {
    return std::cos(x);
}

inline double tan(double x) {
    return std::tan(x);
}

inline double asin(double x) {
    return std::asin(x);
}

inline double acos(double x) {
    return std::acos(x);
}

inline double atan(double x) {
    return std::atan(x);
}

inline double atan2(double y, double x) {
    return std::atan2(y, x);
}

inline double pow(double x, double y) {
    return std::pow(x, y);
}

inline double exp(double x) {
    return std::exp(x);
}

inline double log(double x) {
    return std::log(x);
}

inline double log10(double x) {
    return std::log10(x);
}

inline double floor(double x) {
    return std::floor(x);
}

inline double ceil(double x) {
    return std::ceil(x);
}

inline double round(double x) {
    return std::round(x);
}

inline long random(long max) {
    return rand() % max;
}

inline long random(long min, long max) {
    return min + rand() % (max - min);
}

inline void randomSeed(unsigned long seed) {
    srand(seed);
}

// Delay functions (moved from HALConfig.h to avoid threading dependencies)
inline void delay(unsigned long ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

inline void delayMicroseconds(unsigned long us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}

// Forward declarations for HAL-based Arduino functions
// These will be implemented in ArduinoCompat.cpp to use HAL interfaces
void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
int digitalRead(int pin);
int analogRead(int pin);
void analogWrite(int pin, int value);
void analogReadResolution(int bits);
unsigned long micros();
unsigned long millis();

// Serial class for simulation
#include <iostream>
class SerialClass {
public:
    void begin(unsigned long baud) {
        std::cout << "Serial initialized at " << baud << " baud" << std::endl;
    }

    void print(const char* str) {
        std::cout << str;
    }

    void print(int val) {
        std::cout << val;
    }

    void print(double val) {
        std::cout << val;
    }

    void println(const char* str) {
        std::cout << str << std::endl;
    }

    void println(int val) {
        std::cout << val << std::endl;
    }

    void println(double val) {
        std::cout << val << std::endl;
    }

    void println() {
        std::cout << std::endl;
    }
};

extern SerialClass Serial;

// String class compatibility
class String {
private:
    std::string m_str;

public:
    String() = default;
    String(const char* str) : m_str(str) {}
    String(const std::string& str) : m_str(str) {}
    String(int value) : m_str(std::to_string(value)) {}
    String(double value) : m_str(std::to_string(value)) {}

    const char* c_str() const { return m_str.c_str(); }
    size_t length() const { return m_str.length(); }

    String operator+(const String& other) const {
        return String(m_str + other.m_str);
    }

    String& operator+=(const String& other) {
        m_str += other.m_str;
        return *this;
    }

    bool operator==(const String& other) const {
        return m_str == other.m_str;
    }

    char operator[](size_t index) const {
        return m_str[index];
    }

    char& operator[](size_t index) {
        return m_str[index];
    }
};

#endif // HAL_IMPLEMENTATION == HAL_SIM

#endif // _ARDUINO_COMPAT_H_
