#ifndef _ICAN_BUS_H_
#define _ICAN_BUS_H_

#include "HALConfig.h"

/**
 * @brief CAN Message structure for cross-platform compatibility
 */
struct CANMessage {
    uint32_t id;
    uint8_t len;
    bool extended;
    uint8_t data[8];
    
    CANMessage() : id(0), len(0), extended(false) {
        memset(data, 0, sizeof(data));
    }
};

/**
 * @brief Abstract interface for CAN Bus communication
 * 
 * This interface abstracts CAN bus operations to allow for both
 * real hardware (FlexCAN_T4) and simulation implementations.
 */
class ICANBus {
public:
    virtual ~ICANBus() = default;
    
    /**
     * @brief Initialize the CAN bus
     * @param baudRate Baud rate for CAN communication
     * @return true if initialization successful
     */
    virtual bool begin(uint32_t baudRate = 1000000) = 0;
    
    /**
     * @brief Send a CAN message
     * @param msg The message to send
     * @return true if message was sent successfully
     */
    virtual bool write(const CANMessage& msg) = 0;
    
    /**
     * @brief Check if a message is available to read
     * @return true if message is available
     */
    virtual bool available() = 0;
    
    /**
     * @brief Read a CAN message
     * @param msg Reference to store the received message
     * @return true if message was read successfully
     */
    virtual bool read(CANMessage& msg) = 0;
    
    /**
     * @brief Set the baud rate
     * @param baudRate New baud rate
     */
    virtual void setBaudRate(uint32_t baudRate) = 0;
    
    /**
     * @brief Get the current baud rate
     * @return Current baud rate
     */
    virtual uint32_t getBaudRate() const = 0;
    
    /**
     * @brief Enable/disable debug printing
     * @param enable True to enable debug output
     */
    virtual void setDebugMode(bool enable) = 0;
};

#endif // _ICAN_BUS_H_
