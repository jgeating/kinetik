#ifndef _SIM_CAN_BUS_H_
#define _SIM_CAN_BUS_H_

#include "../HALConfig.h"

#if HAL_IMPLEMENTATION == HAL_SIM

#include "../ICANBus.h"
#include <queue>
#include <mutex>
#include <thread>

/**
 * @brief Simulation implementation of CAN bus
 * 
 * This implementation simulates CAN bus communication by:
 * - Storing outgoing messages in a queue for the simulation server
 * - Receiving incoming messages from the simulation server
 * - Providing realistic timing and behavior
 */
class SimCANBus : public ICANBus {
private:
    int m_busNumber;
    uint32_t m_baudRate;
    bool m_debugMode;
    bool m_initialized;
    
    // Message queues
    std::queue<CANMessage> m_txQueue;
    std::queue<CANMessage> m_rxQueue;
    
    // Thread safety
    mutable std::mutex m_txMutex;
    mutable std::mutex m_rxMutex;
    
    // Statistics
    uint32_t m_messagesSent;
    uint32_t m_messagesReceived;
    uint32_t m_errorsCount;

public:
    /**
     * @brief Constructor
     * @param busNumber CAN bus number (for multi-bus systems)
     */
    explicit SimCANBus(int busNumber = 0);
    
    /**
     * @brief Destructor
     */
    ~SimCANBus();
    
    // ICANBus interface implementation
    bool begin(uint32_t baudRate = 1000000) override;
    bool write(const CANMessage& msg) override;
    bool available() override;
    bool read(CANMessage& msg) override;
    void setBaudRate(uint32_t baudRate) override;
    uint32_t getBaudRate() const override;
    void setDebugMode(bool enable) override;
    
    // Simulation-specific methods
    
    /**
     * @brief Inject a message from the simulation (for testing)
     * @param msg Message to inject
     */
    void injectMessage(const CANMessage& msg);
    
    /**
     * @brief Get all pending outgoing messages
     * @return Vector of messages to send to simulation
     */
    std::vector<CANMessage> getPendingMessages();
    
    /**
     * @brief Get statistics
     */
    void getStatistics(uint32_t& sent, uint32_t& received, uint32_t& errors) const;
    
    /**
     * @brief Reset statistics
     */
    void resetStatistics();
    
    /**
     * @brief Simulate a CAN bus error
     */
    void simulateError();

private:
    /**
     * @brief Log debug message if debug mode is enabled
     * @param message Message to log
     */
    void debugLog(const std::string& message);
    
    /**
     * @brief Convert CANMessage to JSON for simulation communication
     * @param msg CAN message
     * @return JSON representation
     */
    nlohmann::json messageToJson(const CANMessage& msg);
    
    /**
     * @brief Convert JSON to CANMessage from simulation communication
     * @param json JSON representation
     * @return CAN message
     */
    CANMessage jsonToMessage(const nlohmann::json& json);
};

#endif // HAL_IMPLEMENTATION == HAL_SIM

#endif // _SIM_CAN_BUS_H_
