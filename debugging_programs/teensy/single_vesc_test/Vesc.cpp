#include "Vesc.h"
#include "utils.h"

// origin vesc code in robot_code\swerve_master\src\Drive.cpp

Vesc::Vesc(const FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can, const int canId)
  : m_can{ can }, m_canId{ canId } {
  m_msg.flags.extended = true;
}

void Vesc::setAbsolutePosition(float position) {
 
}

void Vesc::setPosition(float position) {
  
}

void Vesc::setVelocity(double vel) {
  int pole_pairs = 7;
  double dRatio = pole_pairs * 60 / (2 * M_PI) / (.083 / 2);

  m_msg.id = m_canId | CAN_PACKET_SET_RPM << 8;
  m_msg.len = 4;
  for (int m = 0; m < 4; m++) {
    m_msg.buf[4 - m - 1] = (int)(vel * dRatio) >> 8 * m;
  }
  write();
}

void Vesc::setAxisState(uint32_t state) {
 
}

void Vesc::disable() {

}

void Vesc::enableWithClosedLoop() {
}

void Vesc::enablePrintOnWrite() {
  m_printMessageOnWrite = true;
}

void Vesc::disablePrintOnWrite() {
  m_printMessageOnWrite = false;
}

void Vesc::printMessage() {
  for (uint8_t i = 0; i < m_msg.len; i++) {
    Serial.print(m_msg.buf[i], HEX);
    if (i < m_msg.len - 1) {
      Serial.print(", ");
    } else {
      Serial.println();
    }
  }
}

void Vesc::write() {
  if (m_printMessageOnWrite) {
    printMessage();
  }
  m_can.write(m_msg);
}

void Vesc::setControlMode(uint32_t controlMode, uint32_t inputMode) {
  
}

void Vesc::setPositionControlMode() {

}

void Vesc::setVelocityControlMode() {

}

void Vesc::getEncoderValues(float& position, float& velocity) {
 
}

float Vesc::getEncoderVelocity() {
  return 0.0;
}

float Vesc::getEncoderPosition() {
  return 0.0;
}
