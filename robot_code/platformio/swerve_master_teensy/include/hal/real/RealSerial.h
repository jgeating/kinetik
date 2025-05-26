#ifndef _REAL_SERIAL_H_
#define _REAL_SERIAL_H_

#include "../HALConfig.h"

#if HAL_IMPLEMENTATION == HAL_REAL

#include "../ISerial.h"
#include <Arduino.h>

/**
 * @brief Real hardware implementation of Serial using Arduino Serial
 */
class RealSerial : public ISerial {
private:
    HardwareSerial* m_serial;
    int m_portNumber;
    bool m_initialized;

public:
    /**
     * @brief Constructor
     * @param portNumber Serial port number (0 = USB Serial, 1 = Serial1, etc.)
     */
    explicit RealSerial(int portNumber = 0);
    
    // ISerial interface implementation
    void begin(unsigned long baudRate) override;
    void end() override;
    bool isReady() override;
    int available() override;
    int read() override;
    size_t readBytes(uint8_t* buffer, size_t length) override;
    size_t write(uint8_t data) override;
    size_t write(const uint8_t* buffer, size_t length) override;
    size_t write(const char* str) override;
    size_t print(const char* str) override;
    size_t println(const char* str) override;
    size_t print(int value, int base = 10) override;
    size_t print(double value, int digits = 2) override;
    size_t println(int value, int base = 10) override;
    size_t println(double value, int digits = 2) override;
    void flush() override;
    void setTimeout(unsigned long timeout) override;

private:
    /**
     * @brief Get the appropriate HardwareSerial instance
     * @param portNumber Port number
     * @return Pointer to HardwareSerial instance
     */
    HardwareSerial* getSerialInstance(int portNumber);
};

#endif // HAL_IMPLEMENTATION == HAL_REAL

#endif // _REAL_SERIAL_H_
