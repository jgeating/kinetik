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

// Arduino-style type definitions
typedef uint8_t byte;
typedef bool boolean;

// Arduino constants
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

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

inline double abs(double x) {
    return std::abs(x);
}

inline long abs(long x) {
    return std::abs(x);
}

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

// Mock Arduino functions that don't apply in simulation
inline void pinMode(int pin, int mode) {
    // No-op in simulation
}

inline void digitalWrite(int pin, int value) {
    // No-op in simulation
}

inline int digitalRead(int pin) {
    // Return default value in simulation
    return LOW;
}

inline int analogRead(int pin) {
    // Return default value in simulation
    return 0;
}

inline void analogWrite(int pin, int value) {
    // No-op in simulation
}

inline void analogReadResolution(int bits) {
    // No-op in simulation
}

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
