#ifndef	__ODRIVE_H
#define	__ODRIVE_H

#include <cstdint>

/**
 * @brief Platform-agnostic ODrive motor controller interface
 *
 * This header defines the interface for ODrive motor controllers.
 * Implementation details are handled by platform-specific .cpp files:
 * - ODrive.cpp: Real hardware implementation (uses FlexCAN_T4)
 * - ODrive_sim.cpp: Simulation implementation (uses HAL)
 */

class ODrive {
public:
  /**
   * @brief Constructor for ODrive controller
   * @param canId CAN ID of the ODrive controller
   */
  ODrive(int canId);

  // Position control
  void setPosition(float position);
  void setAbsolutePosition(float position);

  // Velocity control
  void setVelocity(float revPerSec);

  // State control
  void disable();
  void enableWithClosedLoop();

  // Control modes
  void setVelocityControlMode();
  void setPositionControlMode();

  // Encoder feedback
  void getEncoderValues(float& position, float& velocity);
  float getEncoderPosition();
  float getEncoderVelocity();

  // Debug/diagnostics
  void enablePrintOnWrite();
  void disablePrintOnWrite();
  void printMessage();

private:
  int m_canId;
  bool m_printMessageOnWrite = false;

  // Internal implementation methods
  void setAxisState(uint32_t state);
  void setControlMode(uint32_t controlMode, uint32_t inputMode);
  void writeMessage();
};

/********************************************************************/
#endif // __ODRIVE_H
