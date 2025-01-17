#include "ODrive.h"

ODrive::ODrive(const FlexCAN_T4<CANBUS, RX_SIZE_256, TX_SIZE_16>& can, const int canId)
  : m_can{ can }, m_canId{ canId } {
  m_msg.flags.extended = false;
  m_encoderEstimateMsg.flags.extended = false;
}

void ODrive::setAbsolutePosition(float position) {
  m_msg.id = m_canId << 5 | 0x19;
  m_msg.len = 4;
  memcpy(m_msg.buf, &position, sizeof(position));
  write();
}

void ODrive::setPosition(float position) {
  float torqueFF = 0.0;

  m_msg.id = m_canId << 5 | 0x0c;
  m_msg.len = 8;
  memcpy(m_msg.buf, &position, sizeof(position));
  memcpy(m_msg.buf + sizeof(position), &torqueFF, sizeof(torqueFF));
  write();
}

void ODrive::setVelocity(float revPerSec) {
  float torqueFF = 0.0;

  m_msg.id = m_canId << 5 | 0x0d;
  m_msg.len = 8;
  memcpy(m_msg.buf, &revPerSec, sizeof(revPerSec));
  memcpy(m_msg.buf + sizeof(revPerSec), &torqueFF, sizeof(torqueFF));
  write();
}

void ODrive::setAxisState(uint32_t state) {
  m_msg.id = m_canId << 5 | 0x07;
  m_msg.len = 4;
  memcpy(m_msg.buf, &state, sizeof(state));
  write();
}

void ODrive::disable() {
  setAxisState(1);
}

void ODrive::enableWithClosedLoop() {
  setAxisState(8);
}

void ODrive::enablePrintOnWrite() {
  m_printMessageOnWrite = true;
}

void ODrive::disablePrintOnWrite() {
  m_printMessageOnWrite = false;
}

void ODrive::printMessage() {
  for (uint8_t i = 0; i < m_msg.len; i++) {
    Serial.print(m_msg.buf[i], HEX);
    if (i < m_msg.len - 1) {
      Serial.print(", ");
    } else {
      Serial.println();
    }
  }
}

void ODrive::write() {
  if (m_printMessageOnWrite) {
    printMessage();
  }
  m_can.write(m_msg);
}

void ODrive::setControlMode(uint32_t controlMode, uint32_t inputMode) {
  m_msg.id = m_canId << 5 | 0x0b;
  m_msg.len = 8;
  memcpy(m_msg.buf, &controlMode, sizeof(controlMode));
  memcpy(m_msg.buf + sizeof(controlMode), &inputMode, sizeof(inputMode));
  write();
}

void ODrive::setPositionControlMode() {
  setControlMode(3, 3);
}

void ODrive::setVelocityControlMode() {
  setControlMode(2, 2);
}

void ODrive::getEncoderValues(float& position, float& velocity) {
  m_encoderEstimateMsg.id = m_canId << 5 | 0x09;
  m_encoderEstimateMsg.len = 8;
  m_can.read(m_encoderEstimateMsg);
  memcpy(&position, m_encoderEstimateMsg.buf, sizeof(position));
  memcpy(&velocity, m_encoderEstimateMsg.buf + sizeof(position), sizeof(velocity));
}

float ODrive::getEncoderVelocity() {
  float velocity;
  float position;
  getEncoderValues(position, velocity);
  return velocity;
}

float ODrive::getEncoderPosition() {
  float velocity;
  float position;
  getEncoderValues(position, velocity);
  return position;
}
