#ifndef _ISERIAL_H_
#define _ISERIAL_H_

#include "HALConfig.h"

/**
 * @brief Abstract interface for Serial Communication
 * 
 * This interface abstracts serial operations to allow for both
 * real hardware (Arduino Serial) and simulation implementations.
 */
class ISerial {
public:
    virtual ~ISerial() = default;
    
    /**
     * @brief Initialize serial communication
     * @param baudRate Baud rate for communication
     */
    virtual void begin(unsigned long baudRate) = 0;
    
    /**
     * @brief End serial communication
     */
    virtual void end() = 0;
    
    /**
     * @brief Check if serial is ready for communication
     * @return true if ready
     */
    virtual bool isReady() = 0;
    
    /**
     * @brief Get number of bytes available to read
     * @return Number of available bytes
     */
    virtual int available() = 0;
    
    /**
     * @brief Read a single byte
     * @return Byte read, or -1 if no data available
     */
    virtual int read() = 0;
    
    /**
     * @brief Read multiple bytes into buffer
     * @param buffer Buffer to store data
     * @param length Maximum number of bytes to read
     * @return Number of bytes actually read
     */
    virtual size_t readBytes(uint8_t* buffer, size_t length) = 0;
    
    /**
     * @brief Write a single byte
     * @param data Byte to write
     * @return Number of bytes written (0 or 1)
     */
    virtual size_t write(uint8_t data) = 0;
    
    /**
     * @brief Write multiple bytes
     * @param buffer Buffer containing data to write
     * @param length Number of bytes to write
     * @return Number of bytes actually written
     */
    virtual size_t write(const uint8_t* buffer, size_t length) = 0;
    
    /**
     * @brief Write a string
     * @param str String to write
     * @return Number of bytes written
     */
    virtual size_t write(const char* str) = 0;
    
    /**
     * @brief Print a string
     * @param str String to print
     * @return Number of bytes written
     */
    virtual size_t print(const char* str) = 0;
    
    /**
     * @brief Print a string with newline
     * @param str String to print
     * @return Number of bytes written
     */
    virtual size_t println(const char* str) = 0;
    
    /**
     * @brief Print an integer
     * @param value Integer to print
     * @param base Number base (default 10)
     * @return Number of bytes written
     */
    virtual size_t print(int value, int base = 10) = 0;
    
    /**
     * @brief Print a float
     * @param value Float to print
     * @param digits Number of decimal places
     * @return Number of bytes written
     */
    virtual size_t print(double value, int digits = 2) = 0;
    
    /**
     * @brief Print an integer with newline
     * @param value Integer to print
     * @param base Number base (default 10)
     * @return Number of bytes written
     */
    virtual size_t println(int value, int base = 10) = 0;
    
    /**
     * @brief Print a float with newline
     * @param value Float to print
     * @param digits Number of decimal places
     * @return Number of bytes written
     */
    virtual size_t println(double value, int digits = 2) = 0;
    
    /**
     * @brief Flush output buffer
     */
    virtual void flush() = 0;
    
    /**
     * @brief Set timeout for read operations
     * @param timeout Timeout in milliseconds
     */
    virtual void setTimeout(unsigned long timeout) = 0;
};

#endif // _ISERIAL_H_
