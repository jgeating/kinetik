/**
 * @file ODrive.cpp
 * @brief Real hardware implementation of ODrive using FlexCAN_T4
 */

#include "ODrive.h"
#include <FlexCAN_T4.h>
#include "Constants.h"

// Private implementation details for real hardware
class ODriveImpl {
public:
  CAN_message_t m_msg;
  CAN_message_t m_encoderEstimateMsg;

  ODriveImpl() {
    m_msg.flags.extended = false;
    m_encoderEstimateMsg.flags.extended = false;
  }

  // Get reference to the CAN bus (defined in Swerve.h)
  FlexCAN_T4<CANBUS, RX_SIZE_256, TX_SIZE_16>& getCAN();
};

static ODriveImpl* impl = nullptr;

ODrive::ODrive(int canId) : m_canId(canId), m_printMessageOnWrite(false) {
  if (!impl) {
    impl = new ODriveImpl();
  }
}

void ODrive::setAbsolutePosition(float position) {
  impl->m_msg.id = m_canId << 5 | 0x19;
  impl->m_msg.len = 4;
  memcpy(impl->m_msg.buf, &position, sizeof(position));
  writeMessage();
}

void ODrive::setPosition(float position) {
  float torqueFF = 0.0;

  impl->m_msg.id = m_canId << 5 | 0x0c;
  impl->m_msg.len = 8;
  memcpy(impl->m_msg.buf, &position, sizeof(position));
  memcpy(impl->m_msg.buf + sizeof(position), &torqueFF, sizeof(torqueFF));
  writeMessage();
}

void ODrive::setVelocity(float revPerSec) {
  float torqueFF = 0.0;

  impl->m_msg.id = m_canId << 5 | 0x0d;
  impl->m_msg.len = 8;
  memcpy(impl->m_msg.buf, &revPerSec, sizeof(revPerSec));
  memcpy(impl->m_msg.buf + sizeof(revPerSec), &torqueFF, sizeof(torqueFF));
  writeMessage();
}

void ODrive::setAxisState(uint32_t state) {
  impl->m_msg.id = m_canId << 5 | 0x07;
  impl->m_msg.len = 4;
  memcpy(impl->m_msg.buf, &state, sizeof(state));
  writeMessage();
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
  for (uint8_t i = 0; i < impl->m_msg.len; i++) {
    Serial.print(impl->m_msg.buf[i], HEX);
    if (i < impl->m_msg.len - 1) {
      Serial.print(", ");
    } else {
      Serial.println();
    }
  }
}

void ODrive::writeMessage() {
  if (m_printMessageOnWrite) {
    printMessage();
  }
  impl->getCAN().write(impl->m_msg);
}

// Implementation of getCAN() - access the global CAN bus from Swerve.h
#include "Swerve.h"
FlexCAN_T4<CANBUS, RX_SIZE_256, TX_SIZE_16>& ODriveImpl::getCAN() {
  return motors::canBus1;
}

void ODrive::setControlMode(uint32_t controlMode, uint32_t inputMode) {
  impl->m_msg.id = m_canId << 5 | 0x0b;
  impl->m_msg.len = 8;
  memcpy(impl->m_msg.buf, &controlMode, sizeof(controlMode));
  memcpy(impl->m_msg.buf + sizeof(controlMode), &inputMode, sizeof(inputMode));
  writeMessage();
}

void ODrive::setPositionControlMode() {
  setControlMode(3, 3);
}

void ODrive::setVelocityControlMode() {
  setControlMode(2, 2);
}

void ODrive::getEncoderValues(float& position, float& velocity) {
  impl->m_encoderEstimateMsg.id = m_canId << 5 | 0x09;
  impl->m_encoderEstimateMsg.len = 8;
  impl->getCAN().read(impl->m_encoderEstimateMsg);
  memcpy(&position, impl->m_encoderEstimateMsg.buf, sizeof(position));
  memcpy(&velocity, impl->m_encoderEstimateMsg.buf + sizeof(position), sizeof(velocity));
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
