#ifndef _SIM_IMU_H_
#define _SIM_IMU_H_

#include "../HALConfig.h"

#if HAL_IMPLEMENTATION == HAL_SIM

#include "../IIMU.h"
#include <mutex>
#include <random>

/**
 * @brief Simulation implementation of IMU
 * 
 * This implementation simulates IMU behavior by:
 * - Providing realistic sensor data with configurable noise
 * - Allowing external control via simulation interface
 * - Simulating calibration states and sensor drift
 */
class SimIMU : public IIMU {
private:
    uint8_t m_address;
    bool m_initialized;
    bool m_connected;
    bool m_useExternalCrystal;
    
    // Simulated sensor data
    Vector3D m_eulerAngles;
    Vector3D m_angularVelocity;
    Vector3D m_linearAcceleration;
    Vector3D m_magnetometer;
    Quaternion m_quaternion;
    int8_t m_temperature;
    IMUCalibration m_calibration;
    
    // Noise simulation
    std::mt19937 m_rng;
    std::normal_distribution<double> m_noiseDist;
    
    // Thread safety
    mutable std::mutex m_dataMutex;
    
    // Simulation parameters
    double m_noiseLevel;
    bool m_enableNoise;
    unsigned long m_lastUpdateTime;

public:
    /**
     * @brief Constructor
     * @param address I2C address (for compatibility)
     */
    explicit SimIMU(uint8_t address = 0x28);
    
    // IIMU interface implementation
    bool begin() override;
    Vector3D getEulerAngles() override;
    Vector3D getAngularVelocity() override;
    Vector3D getLinearAcceleration() override;
    Vector3D getMagnetometer() override;
    Quaternion getQuaternion() override;
    int8_t getTemperature() override;
    IMUCalibration getCalibrationStatus() override;
    bool isConnected() override;
    void setExternalCrystal(bool useExternal) override;
    
    // Simulation-specific methods
    
    /**
     * @brief Set simulated Euler angles
     * @param angles New Euler angles
     */
    void setEulerAngles(const Vector3D& angles);
    
    /**
     * @brief Set simulated angular velocity
     * @param velocity New angular velocity
     */
    void setAngularVelocity(const Vector3D& velocity);
    
    /**
     * @brief Set simulated linear acceleration
     * @param acceleration New linear acceleration
     */
    void setLinearAcceleration(const Vector3D& acceleration);
    
    /**
     * @brief Set simulated magnetometer data
     * @param mag New magnetometer data
     */
    void setMagnetometer(const Vector3D& mag);
    
    /**
     * @brief Set simulated temperature
     * @param temp New temperature
     */
    void setTemperature(int8_t temp);
    
    /**
     * @brief Set calibration status
     * @param cal New calibration status
     */
    void setCalibrationStatus(const IMUCalibration& cal);
    
    /**
     * @brief Enable/disable sensor noise
     * @param enable True to enable noise
     * @param level Noise level (standard deviation)
     */
    void setNoiseEnabled(bool enable, double level = 0.01);
    
    /**
     * @brief Simulate connection loss
     * @param connected True if connected
     */
    void setConnected(bool connected);
    
    /**
     * @brief Get all sensor data as JSON for simulation communication
     * @return JSON representation of all sensor data
     */
    nlohmann::json toJson() const;
    
    /**
     * @brief Update sensor data from JSON (from simulation)
     * @param json JSON data from simulation
     */
    void fromJson(const nlohmann::json& json);

private:
    /**
     * @brief Add noise to a value
     * @param value Original value
     * @return Value with noise added
     */
    double addNoise(double value);
    
    /**
     * @brief Add noise to a vector
     * @param vec Original vector
     * @return Vector with noise added
     */
    Vector3D addNoise(const Vector3D& vec);
    
    /**
     * @brief Update quaternion from Euler angles
     */
    void updateQuaternionFromEuler();
    
    /**
     * @brief Simulate realistic sensor behavior
     */
    void updateSimulation();
};

#endif // HAL_IMPLEMENTATION == HAL_SIM

#endif // _SIM_IMU_H_
