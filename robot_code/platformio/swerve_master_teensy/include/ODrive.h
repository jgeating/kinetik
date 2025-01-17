#ifndef	__ODRIVE_H
#define	__ODRIVE_H

#include <FlexCAN_T4.h>
#include "Constants.h"

class ODrive {

public:
  ODrive(const FlexCAN_T4<CANBUS, RX_SIZE_256, TX_SIZE_16>& can, const int canId);
  void setPosition(float position);
  void setAbsolutePosition(float position);
  void setVelocity(float revPerSec);
  void disable();
  void enableWithClosedLoop();
  void enablePrintOnWrite();
  void disablePrintOnWrite();
  void printMessage();
  void setVelocityControlMode();
  void setPositionControlMode();
  void getEncoderValues(float& position, float& velocity);
  float getEncoderPosition();
  float getEncoderVelocity();

private:
  const FlexCAN_T4<CANBUS, RX_SIZE_256, TX_SIZE_16>& m_can;
  const int m_canId;
  CAN_message_t m_msg;
  CAN_message_t m_encoderEstimateMsg;
  bool m_printMessageOnWrite = false;

  void setAxisState(uint32_t);
  void write();
  void setControlMode(uint32_t controlMode, uint32_t inputMode);
};


/********************************************************************/
#endif // __ODRIVE_H
