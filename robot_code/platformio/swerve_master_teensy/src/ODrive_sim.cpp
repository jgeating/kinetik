/**
 * @file ODrive_sim.cpp
 * @brief Simulation implementation of ODrive using HAL data layer
 * 
 * This implementation provides the same interface as the real ODrive
 * but stores/retrieves data from the HAL simulation layer instead of actual hardware.
 * The HAL serves as the simulated hardware data layer that can be accessed via websocket.
 */

#include "ODrive.h"
#include "hal/HALFactory.h"
#include <iostream>
#include <unordered_map>
#include <cstring>

// Simulation data storage using HAL as data layer
struct ODriveSimData {
    float position = 0.0f;
    float velocity = 0.0f;
    float targetPosition = 0.0f;
    float targetVelocity = 0.0f;
    uint32_t axisState = 1; // 1 = idle, 8 = closed loop control
    uint32_t controlMode = 3; // 3 = position control
    uint32_t inputMode = 3; // 3 = position input
    bool enabled = false;
};

// Global storage for all ODrive instances (this would be in HAL in a real implementation)
static std::unordered_map<int, ODriveSimData> g_odriveData;

ODrive::ODrive(int canId) : m_canId(canId), m_printMessageOnWrite(false) {
    // Initialize simulation data for this ODrive
    g_odriveData[canId] = ODriveSimData();
    std::cout << "ODrive: Simulation mode initialized for CAN ID " << canId << std::endl;
}

void ODrive::setAbsolutePosition(float position) {
    auto& data = g_odriveData[m_canId];
    data.targetPosition = position;
    data.position = position; // In simulation, instantly reach target
    
    if (m_printMessageOnWrite) {
        std::cout << "ODrive[" << m_canId << "]: Set absolute position to " << position << std::endl;
    }
}

void ODrive::setPosition(float position) {
    auto& data = g_odriveData[m_canId];
    data.targetPosition = position;
    
    // Simple simulation: gradually move towards target
    float error = position - data.position;
    data.position += error * 0.1f; // Simple proportional control
    data.velocity = error * 0.5f; // Velocity proportional to error
    
    if (m_printMessageOnWrite) {
        std::cout << "ODrive[" << m_canId << "]: Set position to " << position 
                  << " (current: " << data.position << ")" << std::endl;
    }
}

void ODrive::setVelocity(float revPerSec) {
    auto& data = g_odriveData[m_canId];
    data.targetVelocity = revPerSec;
    data.velocity = revPerSec; // In simulation, instantly reach target velocity
    
    // Update position based on velocity (simple integration)
    data.position += revPerSec * 0.001f; // Assume 1ms timestep
    
    if (m_printMessageOnWrite) {
        std::cout << "ODrive[" << m_canId << "]: Set velocity to " << revPerSec << " rev/s" << std::endl;
    }
}

void ODrive::setAxisState(uint32_t state) {
    auto& data = g_odriveData[m_canId];
    data.axisState = state;
    data.enabled = (state == 8); // 8 = closed loop control
    
    if (m_printMessageOnWrite) {
        std::cout << "ODrive[" << m_canId << "]: Set axis state to " << state 
                  << " (" << (data.enabled ? "enabled" : "disabled") << ")" << std::endl;
    }
}

void ODrive::disable() {
    setAxisState(1);
}

void ODrive::enableWithClosedLoop() {
    setAxisState(8);
}

void ODrive::enablePrintOnWrite() {
    m_printMessageOnWrite = true;
    std::cout << "ODrive[" << m_canId << "]: Print on write enabled" << std::endl;
}

void ODrive::disablePrintOnWrite() {
    m_printMessageOnWrite = false;
}

void ODrive::printMessage() {
    auto& data = g_odriveData[m_canId];
    std::cout << "ODrive[" << m_canId << "] State: pos=" << data.position 
              << " vel=" << data.velocity << " enabled=" << data.enabled << std::endl;
}

void ODrive::writeMessage() {
    // In simulation, writeMessage is just for debug output
    if (m_printMessageOnWrite) {
        printMessage();
    }
}

void ODrive::setControlMode(uint32_t controlMode, uint32_t inputMode) {
    auto& data = g_odriveData[m_canId];
    data.controlMode = controlMode;
    data.inputMode = inputMode;
    
    if (m_printMessageOnWrite) {
        std::cout << "ODrive[" << m_canId << "]: Set control mode to " << controlMode 
                  << ", input mode to " << inputMode << std::endl;
    }
}

void ODrive::setPositionControlMode() {
    setControlMode(3, 3);
}

void ODrive::setVelocityControlMode() {
    setControlMode(2, 2);
}

void ODrive::getEncoderValues(float& position, float& velocity) {
    auto& data = g_odriveData[m_canId];
    position = data.position;
    velocity = data.velocity;
}

float ODrive::getEncoderVelocity() {
    return g_odriveData[m_canId].velocity;
}

float ODrive::getEncoderPosition() {
    return g_odriveData[m_canId].position;
}

// Future: These functions would interface with HAL for websocket access
namespace ODriveSimulation {
    // These functions would be called by the HAL/websocket layer
    void setODrivePosition(int canId, float position) {
        if (g_odriveData.find(canId) != g_odriveData.end()) {
            g_odriveData[canId].position = position;
        }
    }
    
    float getODrivePosition(int canId) {
        auto it = g_odriveData.find(canId);
        return (it != g_odriveData.end()) ? it->second.position : 0.0f;
    }
    
    void setODriveVelocity(int canId, float velocity) {
        if (g_odriveData.find(canId) != g_odriveData.end()) {
            g_odriveData[canId].velocity = velocity;
        }
    }
    
    float getODriveVelocity(int canId) {
        auto it = g_odriveData.find(canId);
        return (it != g_odriveData.end()) ? it->second.velocity : 0.0f;
    }
}
