#include "RMD_M6.h"

RMD_M6::RMD_M6(const FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can, const int motorId)
  : m_can{ can }, m_motorId{ motorId } {
  m_msg.flags.extended = false;
}

void RMD_M6::setPosition(float ang) {
  m_msg.id =  0x141 + this->m_motorId;
  m_msg.len = 4;
  
  ang = ang * 180.0 / PI; // rmd operates with degree as unit
  int32_t ang_int = (int32_t)(ang * 100);  // RMD-X6 takes angle over CAN as int32 in hundredths of degrees
  m_msg.buf[0] = *((uint8_t *)(&ang_int));
  m_msg.buf[1] = *((uint8_t *)(&ang_int)+1);
  m_msg.buf[2] = *((uint8_t *)(&ang_int)+2);
  m_msg.buf[3] = *((uint8_t *)(&ang_int)+3);
  write();
}

void RMD_M6::setVelocity(float revPerSec) {

}

void RMD_M6::enablePrintOnWrite() {
  m_printMessageOnWrite = true;
}

void RMD_M6::disablePrintOnWrite() {
  m_printMessageOnWrite = false;
}

void RMD_M6::printMessage() {
  for (uint8_t i = 0; i < m_msg.len; i++) {
    Serial.print(m_msg.buf[i], HEX);
    if (i < m_msg.len - 1) {
      Serial.print(", ");
    } else {
      Serial.println();
    }
  }
}

void RMD_M6::write() {
  if (m_printMessageOnWrite) {
    printMessage();
  }
  m_can.write(m_msg);
}

float RMD_M6::getEncoderVelocity() {
  return 0;
}

float RMD_M6::getEncoderPosition() {
  return 0;
}
