#ifndef	__ODRIVE_H
#define	__ODRIVE_H

#include <FlexCAN_T4.h>
#include "Constants.h"

// Base class for ODrive motors
class ODriveBase {
public:
  virtual void setPosition(float position) = 0;
  virtual void setAbsolutePosition(float position) = 0;
  virtual void setVelocity(float revPerSec) = 0;
  virtual void disable() = 0;
  virtual void enableWithClosedLoop() = 0;
  virtual void enablePrintOnWrite() = 0;
  virtual void disablePrintOnWrite() = 0;
  virtual void printMessage() = 0;
  virtual void setVelocityControlMode() = 0;
  virtual void setPositionControlMode() = 0;
  virtual void getEncoderValues(float& position, float& velocity) = 0;
  virtual float getEncoderPosition() = 0;
  virtual float getEncoderVelocity() = 0;
  virtual void setEncoderOffset(float offset) = 0;
  virtual float getEncoderOffset() = 0;
  // Called by the main loop with every CAN message read from the bus.
  // The implementation checks whether the message belongs to this node
  // and updates the internal cache if it does.
  virtual void processMessage(const CAN_message_t& msg) = 0;
  virtual ~ODriveBase() = default;
};

template<CAN_DEV_TABLE _bus>
class ODrive : public ODriveBase {

public:
  ODrive(FlexCAN_T4<_bus, RX_SIZE_256, TX_SIZE_16>& can, const int canId);
  void setPosition(float positionRevs) override;
  void setAbsolutePosition(float position) override;
  void setVelocity(float revPerSec) override;
  void disable() override;
  void enableWithClosedLoop() override;
  void enablePrintOnWrite() override;
  void disablePrintOnWrite() override;
  void printMessage() override;
  void setVelocityControlMode() override;
  void setPositionControlMode() override;
  void getEncoderValues(float& position, float& velocity) override;
  float getEncoderPosition() override;
  float getEncoderVelocity() override;
  void setEncoderOffset(float offset) override;
  float getEncoderOffset() override;
  void processMessage(const CAN_message_t& msg) override;

private:
  FlexCAN_T4<_bus, RX_SIZE_256, TX_SIZE_16>& m_can;
  const int m_canId;
  CAN_message_t m_msg;
  bool m_printMessageOnWrite = false;
  float m_encoderOffset = 0.0f;
  float m_cachedPosition = 0.0f;
  float m_cachedVelocity = 0.0f;

  void setAxisState(uint32_t);
  void write();
  void setControlMode(uint32_t controlMode, uint32_t inputMode);
};

// Template implementation
template<CAN_DEV_TABLE _bus>
ODrive<_bus>::ODrive(FlexCAN_T4<_bus, RX_SIZE_256, TX_SIZE_16>& can, const int canId)
  : m_can{ can }, m_canId{ canId } {
  m_msg.flags.extended = false;
  // m_encoderEstimateMsg.flags.extended = false;
}

// Sends a raw position setpoint (in revolutions) directly to the ODrive with no
// offset correction. Used during homing/calibration when the encoder offset has
// not yet been established. Prefer setPosition() for normal closed-loop control.
template<CAN_DEV_TABLE _bus>
void ODrive<_bus>::setAbsolutePosition(float position) {
  m_msg.id = m_canId << 5 | 0x19;
  m_msg.len = 4;
  memcpy(m_msg.buf, &position, sizeof(position));
  write();
}

// Commands a position setpoint (in revolutions)
template<CAN_DEV_TABLE _bus>
void ODrive<_bus>::setPosition(float positionRevs) {
  float torqueFF = 0.0;
  float adjustedPosition = positionRevs + m_encoderOffset;

  m_msg.id = m_canId << 5 | 0x0c;
  m_msg.len = 8;
  memcpy(m_msg.buf, &adjustedPosition, sizeof(adjustedPosition));
  memcpy(m_msg.buf + sizeof(adjustedPosition), &torqueFF, sizeof(torqueFF));
  write();
}

template<CAN_DEV_TABLE _bus>
void ODrive<_bus>::setVelocity(float revPerSec) {
  float torqueFF = 0.0;

  m_msg.id = m_canId << 5 | 0x0d;
  m_msg.len = 8;
  memcpy(m_msg.buf, &revPerSec, sizeof(revPerSec));
  memcpy(m_msg.buf + sizeof(revPerSec), &torqueFF, sizeof(torqueFF));
  write();
}

template<CAN_DEV_TABLE _bus>
void ODrive<_bus>::setAxisState(uint32_t state) {
  m_msg.id = m_canId << 5 | 0x07;
  m_msg.len = 4;
  memcpy(m_msg.buf, &state, sizeof(state));
  write();
}

template<CAN_DEV_TABLE _bus>
void ODrive<_bus>::disable() {
  setAxisState(1);
}

template<CAN_DEV_TABLE _bus>
void ODrive<_bus>::enableWithClosedLoop() {
  setAxisState(8);
}

template<CAN_DEV_TABLE _bus>
void ODrive<_bus>::enablePrintOnWrite() {
  m_printMessageOnWrite = true;
}

template<CAN_DEV_TABLE _bus>
void ODrive<_bus>::disablePrintOnWrite() {
  m_printMessageOnWrite = false;
}

template<CAN_DEV_TABLE _bus>
void ODrive<_bus>::printMessage() {
  for (uint8_t i = 0; i < m_msg.len; i++) {
    Serial.print(m_msg.buf[i], HEX);
    if (i < m_msg.len - 1) {
      Serial.print(", ");
    } else {
      Serial.println();
    }
  }
}

template<CAN_DEV_TABLE _bus>
void ODrive<_bus>::write() {
  if (m_printMessageOnWrite) {
    printMessage();
  }
  m_can.write(m_msg);
}

template<CAN_DEV_TABLE _bus>
void ODrive<_bus>::setControlMode(uint32_t controlMode, uint32_t inputMode) {
  m_msg.id = m_canId << 5 | 0x0b;
  m_msg.len = 8;
  memcpy(m_msg.buf, &controlMode, sizeof(controlMode));
  memcpy(m_msg.buf + sizeof(controlMode), &inputMode, sizeof(inputMode));
  write();
}

template<CAN_DEV_TABLE _bus>
void ODrive<_bus>::setPositionControlMode() {
  setControlMode(3, 3);
}

template<CAN_DEV_TABLE _bus>
void ODrive<_bus>::setVelocityControlMode() {
  setControlMode(2, 2);
}

// Called by the main loop once per message read from the bus.
// Updates the cached position/velocity only if the message ID matches
// this node's Get_Encoder_Estimates frame (cmd 0x09).
template<CAN_DEV_TABLE _bus>
void ODrive<_bus>::processMessage(const CAN_message_t& msg) {
  if (msg.id == static_cast<uint32_t>(m_canId << 5 | 0x09)) {
    memcpy(&m_cachedPosition, msg.buf, sizeof(m_cachedPosition));
    memcpy(&m_cachedVelocity, msg.buf + sizeof(m_cachedPosition), sizeof(m_cachedVelocity));
  }
}

template<CAN_DEV_TABLE _bus>
void ODrive<_bus>::getEncoderValues(float& position, float& velocity) {
  // Values are updated externally via processMessage(); just return the cache.
  position = m_cachedPosition;
  velocity = m_cachedVelocity;
}

// Returns the motor's velocity in rev/s
template<CAN_DEV_TABLE _bus>
float ODrive<_bus>::getEncoderVelocity() {
  float position, velocity;
  getEncoderValues(position, velocity);
  return velocity;
}

// Returns the encoder position in revolutions
template<CAN_DEV_TABLE _bus>
float ODrive<_bus>::getEncoderPosition() {
  float position, velocity;
  getEncoderValues(position, velocity);
  return position - m_encoderOffset;
}

template<CAN_DEV_TABLE _bus>
void ODrive<_bus>::setEncoderOffset(float offset) {
  m_encoderOffset = offset;
}

template<CAN_DEV_TABLE _bus>
float ODrive<_bus>::getEncoderOffset() {
  return m_encoderOffset;
}

/********************************************************************/
#endif // __ODRIVE_H
