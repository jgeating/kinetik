#ifndef _REAL_CAN_BUS_H_
#define _REAL_CAN_BUS_H_

#include "../HALConfig.h"

#if HAL_IMPLEMENTATION == HAL_REAL

#include "../ICANBus.h"
#include <FlexCAN_T4.h>
#include "Constants.h"

/**
 * @brief Real hardware implementation of CAN bus using FlexCAN_T4
 */
class RealCANBus : public ICANBus {
private:
    FlexCAN_T4<CANBUS, RX_SIZE_256, TX_SIZE_16>& m_can;
    uint32_t m_baudRate;
    bool m_debugMode;
    CAN_message_t m_rxMsg;
    CAN_message_t m_txMsg;

public:
    /**
     * @brief Constructor
     * @param can Reference to FlexCAN_T4 instance
     */
    explicit RealCANBus(FlexCAN_T4<CANBUS, RX_SIZE_256, TX_SIZE_16>& can);
    
    // ICANBus interface implementation
    bool begin(uint32_t baudRate = 1000000) override;
    bool write(const CANMessage& msg) override;
    bool available() override;
    bool read(CANMessage& msg) override;
    void setBaudRate(uint32_t baudRate) override;
    uint32_t getBaudRate() const override;
    void setDebugMode(bool enable) override;

private:
    /**
     * @brief Convert HAL CANMessage to FlexCAN CAN_message_t
     * @param halMsg HAL message
     * @param flexMsg FlexCAN message
     */
    void convertToFlexCAN(const CANMessage& halMsg, CAN_message_t& flexMsg);
    
    /**
     * @brief Convert FlexCAN CAN_message_t to HAL CANMessage
     * @param flexMsg FlexCAN message
     * @param halMsg HAL message
     */
    void convertFromFlexCAN(const CAN_message_t& flexMsg, CANMessage& halMsg);
};

#endif // HAL_IMPLEMENTATION == HAL_REAL

#endif // _REAL_CAN_BUS_H_
