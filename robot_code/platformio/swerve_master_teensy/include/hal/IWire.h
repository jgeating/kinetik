#ifndef _IWIRE_H_
#define _IWIRE_H_

#include <stdint.h>

/**
 * @brief Interface for I2C (Wire) communication
 * 
 * This interface provides platform-independent access to I2C communication.
 * It abstracts Arduino Wire library functions.
 */
class IWire {
public:
    virtual ~IWire() = default;
    
    /**
     * @brief Initialize I2C as master
     * @return true if initialization successful
     */
    virtual bool begin() = 0;
    
    /**
     * @brief Initialize I2C as slave with address
     * @param address Slave address (7-bit)
     * @return true if initialization successful
     */
    virtual bool begin(uint8_t address) = 0;
    
    /**
     * @brief Set I2C clock frequency
     * @param frequency Clock frequency in Hz
     */
    virtual void setClock(uint32_t frequency) = 0;
    
    /**
     * @brief Begin transmission to slave device
     * @param address Slave address (7-bit)
     */
    virtual void beginTransmission(uint8_t address) = 0;
    
    /**
     * @brief End transmission and send data
     * @param stop Whether to send stop condition
     * @return 0 = success, other = error code
     */
    virtual uint8_t endTransmission(bool stop = true) = 0;
    
    /**
     * @brief Write a byte to the transmission buffer
     * @param data Byte to write
     * @return Number of bytes written
     */
    virtual size_t write(uint8_t data) = 0;
    
    /**
     * @brief Write multiple bytes to the transmission buffer
     * @param data Pointer to data buffer
     * @param length Number of bytes to write
     * @return Number of bytes written
     */
    virtual size_t write(const uint8_t* data, size_t length) = 0;
    
    /**
     * @brief Request data from slave device
     * @param address Slave address (7-bit)
     * @param quantity Number of bytes to request
     * @param stop Whether to send stop condition
     * @return Number of bytes received
     */
    virtual uint8_t requestFrom(uint8_t address, uint8_t quantity, bool stop = true) = 0;
    
    /**
     * @brief Check if data is available to read
     * @return Number of bytes available
     */
    virtual int available() = 0;
    
    /**
     * @brief Read a byte from the receive buffer
     * @return Byte read, or -1 if no data available
     */
    virtual int read() = 0;
    
    /**
     * @brief Peek at the next byte without removing it
     * @return Next byte, or -1 if no data available
     */
    virtual int peek() = 0;
    
    /**
     * @brief Check if I2C system is ready
     * @return true if ready for operations
     */
    virtual bool isReady() const = 0;
};

#endif // _IWIRE_H_
