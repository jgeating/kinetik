#ifndef _IIMU_H_
#define _IIMU_H_

#include "HALConfig.h"

/**
 * @brief 3D Vector structure for IMU data
 */
struct Vector3D {
    double x, y, z;
    
    Vector3D() : x(0), y(0), z(0) {}
    Vector3D(double x, double y, double z) : x(x), y(y), z(z) {}
};

/**
 * @brief Quaternion structure for orientation
 */
struct Quaternion {
    double w, x, y, z;
    
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}
};

/**
 * @brief IMU calibration status
 */
struct IMUCalibration {
    uint8_t system;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
    
    IMUCalibration() : system(0), gyro(0), accel(0), mag(0) {}
};

/**
 * @brief Abstract interface for IMU (Inertial Measurement Unit)
 * 
 * This interface abstracts IMU operations to allow for both
 * real hardware (BNO055) and simulation implementations.
 */
class IIMU {
public:
    virtual ~IIMU() = default;
    
    /**
     * @brief Initialize the IMU
     * @return true if initialization successful
     */
    virtual bool begin() = 0;
    
    /**
     * @brief Get Euler angles (roll, pitch, yaw)
     * @return Vector3D containing Euler angles in degrees
     */
    virtual Vector3D getEulerAngles() = 0;
    
    /**
     * @brief Get angular velocity (gyroscope data)
     * @return Vector3D containing angular velocity in rad/s
     */
    virtual Vector3D getAngularVelocity() = 0;
    
    /**
     * @brief Get linear acceleration
     * @return Vector3D containing acceleration in m/s²
     */
    virtual Vector3D getLinearAcceleration() = 0;
    
    /**
     * @brief Get magnetometer data
     * @return Vector3D containing magnetic field in µT
     */
    virtual Vector3D getMagnetometer() = 0;
    
    /**
     * @brief Get orientation as quaternion
     * @return Quaternion representing orientation
     */
    virtual Quaternion getQuaternion() = 0;
    
    /**
     * @brief Get IMU temperature
     * @return Temperature in Celsius
     */
    virtual int8_t getTemperature() = 0;
    
    /**
     * @brief Get calibration status
     * @return IMUCalibration structure with calibration values
     */
    virtual IMUCalibration getCalibrationStatus() = 0;
    
    /**
     * @brief Check if IMU is connected and responding
     * @return true if IMU is connected
     */
    virtual bool isConnected() = 0;
    
    /**
     * @brief Set external crystal usage (for BNO055)
     * @param useExternal True to use external crystal
     */
    virtual void setExternalCrystal(bool useExternal) = 0;
};

#endif // _IIMU_H_
