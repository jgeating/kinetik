#ifndef _SIMULATION_INTERFACE_H_
#define _SIMULATION_INTERFACE_H_

#include "../HALConfig.h"

#if HAL_IMPLEMENTATION == HAL_SIM

#include <string>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>

// Forward declarations for simulation components
class SimCANBus;
class SimIMU;
class SimRCReceiver;
class SimSerial;

/**
 * @brief Interface for communicating with simulation server
 * 
 * This class handles WebSocket communication with the simulation server,
 * coordinating data exchange between simulated hardware components and
 * the external simulation environment.
 */
class SimulationInterface {
public:
    /**
     * @brief Callback function type for receiving simulation data
     */
    using MessageCallback = std::function<void(const nlohmann::json&)>;
    
    /**
     * @brief Get singleton instance
     * @return Reference to singleton instance
     */
    static SimulationInterface& getInstance();
    
    /**
     * @brief Initialize the simulation interface
     * @param serverUrl WebSocket server URL (e.g., "ws://localhost:8080")
     * @return true if initialization successful
     */
    bool initialize(const std::string& serverUrl = "ws://localhost:8080");
    
    /**
     * @brief Shutdown the simulation interface
     */
    void shutdown();
    
    /**
     * @brief Check if connected to simulation server
     * @return true if connected
     */
    bool isConnected() const;
    
    /**
     * @brief Send data to simulation server
     * @param data JSON data to send
     * @return true if sent successfully
     */
    bool sendData(const nlohmann::json& data);
    
    /**
     * @brief Register a callback for receiving simulation data
     * @param callback Function to call when data is received
     */
    void setMessageCallback(MessageCallback callback);
    
    /**
     * @brief Register simulation components
     */
    void registerCANBus(SimCANBus* canBus);
    void registerIMU(SimIMU* imu);
    void registerRCReceiver(SimRCReceiver* rcReceiver);
    void registerSerial(SimSerial* serial);
    
    /**
     * @brief Update simulation (call periodically)
     * This sends outgoing data and processes incoming data
     */
    void update();
    
    /**
     * @brief Get connection statistics
     */
    void getStatistics(uint32_t& messagesSent, uint32_t& messagesReceived, 
                      uint32_t& errors, uint32_t& reconnects) const;

private:
    SimulationInterface() = default;
    ~SimulationInterface() = default;
    
    // Non-copyable
    SimulationInterface(const SimulationInterface&) = delete;
    SimulationInterface& operator=(const SimulationInterface&) = delete;
    
    // Connection management
    std::string m_serverUrl;
    std::atomic<bool> m_connected{false};
    std::atomic<bool> m_shouldStop{false};
    
    // Threading
    std::unique_ptr<std::thread> m_networkThread;
    std::mutex m_sendMutex;
    std::mutex m_receiveMutex;
    
    // Callbacks
    MessageCallback m_messageCallback;
    
    // Registered components
    SimCANBus* m_canBus = nullptr;
    SimIMU* m_imu = nullptr;
    SimRCReceiver* m_rcReceiver = nullptr;
    SimSerial* m_serial = nullptr;
    
    // Statistics
    mutable std::mutex m_statsMutex;
    uint32_t m_messagesSent = 0;
    uint32_t m_messagesReceived = 0;
    uint32_t m_errors = 0;
    uint32_t m_reconnects = 0;
    
    // Internal methods
    void networkThreadFunction();
    void processIncomingMessage(const nlohmann::json& message);
    void sendComponentData();
    bool connectToServer();
    void handleConnectionLoss();
    
    // Message handlers
    void handleCANMessage(const nlohmann::json& data);
    void handleIMUMessage(const nlohmann::json& data);
    void handleRCMessage(const nlohmann::json& data);
    void handleSerialMessage(const nlohmann::json& data);
    
    // Utility methods
    nlohmann::json createStatusMessage();
    void logError(const std::string& message);
    void logInfo(const std::string& message);
};

/**
 * @brief RAII helper for simulation interface
 */
class SimulationManager {
public:
    SimulationManager(const std::string& serverUrl = "ws://localhost:8080");
    ~SimulationManager();
    
    bool isReady() const;
    SimulationInterface& getInterface();

private:
    bool m_initialized;
};

#endif // HAL_IMPLEMENTATION == HAL_SIM

#endif // _SIMULATION_INTERFACE_H_
