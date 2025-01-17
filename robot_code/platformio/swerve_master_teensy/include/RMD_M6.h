#ifndef	__RMD_M6_H
#define	__RMD_M6_H

#include <FlexCAN_T4.h>
#include "Constants.h"

class RMD_M6 {

public:
  RMD_M6(const FlexCAN_T4<CANBUS, RX_SIZE_256, TX_SIZE_16>& can, const int motorId);
  void setPosition(float position);
  void setPosition(float position, float velocityDegPerSec);
  void setVelocity(float revPerSec);
  void enablePrintOnWrite();
  void disablePrintOnWrite();
  void printMessage();
  float getEncoderPosition();
  float getEncoderVelocity();

private:
  const FlexCAN_T4<CANBUS, RX_SIZE_256, TX_SIZE_16>& m_can;
  const int m_motorId;
  CAN_message_t m_msg;
  bool m_printMessageOnWrite = false;

  void write();
};


/********************************************************************/
#endif // __RMD_M6
