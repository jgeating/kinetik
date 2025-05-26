#ifndef _REAL_IMU_H_
#define _REAL_IMU_H_

#include "../HALConfig.h"

#if HAL_IMPLEMENTATION == HAL_REAL

#include "../IIMU.h"
#include "Adafruit_BNO055.h"
#include "Adafruit_Sensor.h"
#include "utility/imumaths.h"

/**
 * @brief Real hardware implementation of IMU using Adafruit BNO055
 */
class RealIMU : public IIMU {
private:
    Adafruit_BNO055 m_bno;
    uint8_t m_address;
    bool m_initialized;
    bool m_connected;

public:
    /**
     * @brief Constructor
     * @param address I2C address of the BNO055
     */
    explicit RealIMU(uint8_t address = 0x28);
    
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

private:
    /**
     * @brief Convert imu::Vector<3> to Vector3D
     * @param imuVec IMU library vector
     * @return HAL Vector3D
     */
    Vector3D convertVector(const imu::Vector<3>& imuVec);
    
    /**
     * @brief Convert imu::Quaternion to HAL Quaternion
     * @param imuQuat IMU library quaternion
     * @return HAL Quaternion
     */
    Quaternion convertQuaternion(const imu::Quaternion& imuQuat);
    
    /**
     * @brief Check if the sensor is responding
     * @return true if sensor responds
     */
    bool checkConnection();
};

#endif // HAL_IMPLEMENTATION == HAL_REAL

#endif // _REAL_IMU_H_
