#include "RMD_M6.h"

RMD_M6::RMD_M6(const FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can, const int motorId)
    : m_can{can}, m_motorId{motorId}
{
  m_msg.flags.extended = false;
  m_msg.id = 0x141 + this->m_motorId;
  m_msg.len = 8;
}

void RMD_M6::setPosition(float angleDegrees) {
  setPosition(angleDegrees, 720);
}

void RMD_M6::setPosition(float angleDegrees, float velocityDegPerSec)
{
  m_msg.buf[0] = 0xA4; // Position control command 2
  m_msg.buf[1] = 0;

  uint16_t speedLimit = abs(720) * 18;
  int32_t ang_int = (int32_t)(angleDegrees * 100 * 18); // RMD-X6 takes angle over CAN as int32 in hundredths of degrees
  memcpy(m_msg.buf + 2, &speedLimit, 2);
  memcpy(m_msg.buf + 4, &ang_int, 4);
  write();
}

void RMD_M6::setVelocity(float revPerSec)
{
}

void RMD_M6::enablePrintOnWrite()
{
  m_printMessageOnWrite = true;
}

void RMD_M6::disablePrintOnWrite()
{
  m_printMessageOnWrite = false;
}

void RMD_M6::printMessage()
{
  Serial.print("Motor ");
  Serial.print(m_motorId);
  Serial.print(", ");
  Serial.print(m_msg.id);
  Serial.print(": ");
  for (uint8_t i = 0; i < m_msg.len; i++)
  {
    Serial.print(m_msg.buf[i], HEX);
    if (i < m_msg.len - 1)
    {
      Serial.print(", ");
    }
    else
    {
      Serial.println();
    }
  }
}

void RMD_M6::write()
{
  if (m_printMessageOnWrite)
  {
    printMessage();
  }
  m_can.write(m_msg);
}

float RMD_M6::getEncoderVelocity()
{
  return 0;
}

float RMD_M6::getEncoderPosition()
{
  return 0;
}
