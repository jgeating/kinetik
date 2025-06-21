#ifndef	__VESC_H
#define	__VESC_H

#include <FlexCAN_T4.h>
#include "shared/utils.h"

// Base class for VESC motors
class VescBase {
public:
  virtual void setPosition(float position) = 0;
  virtual void setAbsolutePosition(float position) = 0;
  virtual void setVelocity(double vel) = 0;
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
  virtual ~VescBase() = default;
};

template<CAN_DEV_TABLE _bus>
class Vesc : public VescBase {

public:
  Vesc(const FlexCAN_T4<_bus, RX_SIZE_256, TX_SIZE_16>& can, const int canId);
  void setPosition(float position) override;
  void setAbsolutePosition(float position) override;
  void setVelocity(double vel) override;
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

private:
  const FlexCAN_T4<_bus, RX_SIZE_256, TX_SIZE_16>& m_can;
  const int m_canId;
  CAN_message_t m_msg;
  CAN_message_t m_encoderEstimateMsg;
  bool m_printMessageOnWrite = false;

  void setAxisState(uint32_t);
  void write();
  void setControlMode(uint32_t controlMode, uint32_t inputMode);
};

// Template implementation
template<CAN_DEV_TABLE _bus>
Vesc<_bus>::Vesc(const FlexCAN_T4<_bus, RX_SIZE_256, TX_SIZE_16>& can, const int canId)
  : m_can{ can }, m_canId{ canId } {
  m_msg.flags.extended = true;
}

template<CAN_DEV_TABLE _bus>
void Vesc<_bus>::setAbsolutePosition(float position) {

}

template<CAN_DEV_TABLE _bus>
void Vesc<_bus>::setPosition(float position) {

}

template<CAN_DEV_TABLE _bus>
void Vesc<_bus>::setVelocity(double vel) {
  int pole_pairs = 7;
  double dRatio = pole_pairs * 60 / (2 * M_PI) / (.083 / 2);

  m_msg.id = m_canId | CAN_PACKET_SET_RPM << 8;
  m_msg.len = 4;
  for (int m = 0; m < 4; m++) {
    m_msg.buf[4 - m - 1] = (int)(vel * dRatio) >> 8 * m;
  }
  write();
}

template<CAN_DEV_TABLE _bus>
void Vesc<_bus>::setAxisState(uint32_t state) {

}

template<CAN_DEV_TABLE _bus>
void Vesc<_bus>::disable() {

}

template<CAN_DEV_TABLE _bus>
void Vesc<_bus>::enableWithClosedLoop() {
}

template<CAN_DEV_TABLE _bus>
void Vesc<_bus>::enablePrintOnWrite() {
  m_printMessageOnWrite = true;
}

template<CAN_DEV_TABLE _bus>
void Vesc<_bus>::disablePrintOnWrite() {
  m_printMessageOnWrite = false;
}

template<CAN_DEV_TABLE _bus>
void Vesc<_bus>::printMessage() {
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
void Vesc<_bus>::write() {
  if (m_printMessageOnWrite) {
    printMessage();
  }
  m_can.write(m_msg);
}

template<CAN_DEV_TABLE _bus>
void Vesc<_bus>::setControlMode(uint32_t controlMode, uint32_t inputMode) {

}

template<CAN_DEV_TABLE _bus>
void Vesc<_bus>::setPositionControlMode() {

}

template<CAN_DEV_TABLE _bus>
void Vesc<_bus>::setVelocityControlMode() {

}

template<CAN_DEV_TABLE _bus>
void Vesc<_bus>::getEncoderValues(float& position, float& velocity) {

}

template<CAN_DEV_TABLE _bus>
float Vesc<_bus>::getEncoderVelocity() {
  return 0.0;
}

template<CAN_DEV_TABLE _bus>
float Vesc<_bus>::getEncoderPosition() {
  return 0.0;
}

/********************************************************************/
#endif // __VESC_H
