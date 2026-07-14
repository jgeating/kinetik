#ifndef __MOTEUS_N1_H
#define __MOTEUS_N1_H

#include <math.h>
#include <FlexCAN_T4.h>

/**
 * @brief Base interface for MoteusN1 motor wrappers.
 * Provides a unified API consistent with ODrive and Vesc classes.
 */
class MoteusN1Base {
public:
  virtual void setPosition(float position) = 0;
  virtual void setVelocity(float revPerSec) = 0;
  virtual void enable() = 0;
  virtual void disable() = 0;
  virtual void enablePrintOnWrite() = 0;
  virtual void disablePrintOnWrite() = 0;
  virtual void printMessage() = 0;
  virtual void getEncoderValues(float& position, float& velocity) = 0;
  virtual float getEncoderPosition() = 0;
  virtual float getEncoderVelocity() = 0;
  // Diagnostics (usable through a base pointer)
  virtual bool parseReplyMsg(const CANFD_message_t& rx) = 0;
  virtual void sendQueryMsg() = 0;
  virtual int statusMode() = 0;
  virtual int statusFault() = 0;
  virtual float statusPosition() = 0;
  virtual int canId() const = 0;
  virtual ~MoteusN1Base() = default;
};

/**
 * @brief MoteusN1 adapter wrapper for mjbots Moteus N1 motor controller.
 * 
 * Lightweight wrapper that handles Moteus N1 CAN communication directly
 * via FlexCAN_T4, without relying on the full Moteus library (which uses ACAN_T4).
 * Provides unified interface compatible with ODrive and Vesc wrappers.
 * 
 * @tparam _bus CAN bus type (CAN1, CAN3, etc.)
 * @tparam MotorId CAN ID for this motor (0-127, default 1)
 */
template<CAN_DEV_TABLE _bus, int MotorId = 1>
class MoteusN1 : public MoteusN1Base {
public:
  /**
   * @brief Initialize MoteusN1 motor controller on specified CAN bus.
   * @param can Reference to FlexCAN_T4FD bus instance (CAN-FD capable; CAN3 only on Teensy 4.x)
   * @param canId CAN ID for this motor (0-127)
   */
  MoteusN1(FlexCAN_T4FD<_bus, RX_SIZE_256, TX_SIZE_16>& can, const int canId)
      : m_can(can),
        m_canId(canId),
        m_printOnWrite(false),
        m_lastPosition(0.0f),
        m_lastVelocity(0.0f),
        m_lastError(0) {
  }

  /**
   * @brief Command a target position (position mode, unlimited velocity).
   * @param position Target position in revolutions
   */
  void setPosition(float position) override {
    if (isnan(position)) return;
    sendPositionMode(position, 0.0f);
    m_lastPosition = position;

    if (m_printOnWrite) {
      Serial.print("MoteusN1[");
      Serial.print(m_canId);
      Serial.print("] pos=");
      Serial.println(position, 4);
    }
  }

  /**
   * @brief Command a target velocity (position mode with position=NaN).
   * @param revPerSec Target velocity in revolutions per second
   */
  void setVelocity(float revPerSec) override {
    if (isnan(revPerSec)) return;
    sendPositionMode(NAN, revPerSec);
    m_lastVelocity = revPerSec;

    if (m_printOnWrite) {
      Serial.print("MoteusN1[");
      Serial.print(m_canId);
      Serial.print("] vel=");
      Serial.println(revPerSec, 4);
    }
  }

  /**
   * @brief Enable the motor.
   * Sends a stop command, which clears any latched faults (including
   * position-timeout faults) so subsequent position commands are accepted.
   */
  void enable() override {
    sendStop();
  }

  /**
   * @brief Disable the motor by commanding stopped mode.
   */
  void disable() override {
    sendStop();

    if (m_printOnWrite) {
      Serial.print("MoteusN1[");
      Serial.print(m_canId);
      Serial.println("] STOP");
    }
  }

  /**
   * @brief Enable debug printing of CAN messages on write.
   */
  void enablePrintOnWrite() override {
    m_printOnWrite = true;
  }

  /**
   * @brief Disable debug printing of CAN messages on write.
   */
  void disablePrintOnWrite() override {
    m_printOnWrite = false;
  }

  /**
   * @brief Print current state to Serial.
   */
  void printMessage() override {
    Serial.print("MoteusN1[");
    Serial.print(m_canId);
    Serial.print("]: pos=");
    Serial.print(m_lastPosition, 4);
    Serial.print(" vel=");
    Serial.print(m_lastVelocity, 4);
    Serial.print(" err=");
    Serial.println(m_lastError);
  }

  /**
   * @brief Query encoder position and velocity.
   * @param position Output: current position (revolutions)
   * @param velocity Output: current velocity (rev/sec)
   */
  void getEncoderValues(float& position, float& velocity) override {
    // For now, return last commanded values
    // In a real implementation, this would poll CAN for status messages
    position = m_lastPosition;
    velocity = m_lastVelocity;
  }

  /**
   * @brief Get current encoder position.
   * @return Position in revolutions
   */
  float getEncoderPosition() override {
    return m_lastPosition;
  }

  /**
   * @brief Get current encoder velocity.
   * @return Velocity in revolutions per second
   */
  float getEncoderVelocity() override {
    return m_lastVelocity;
  }

  /// Latest status decoded from a reply frame.
  struct Status {
    int mode = -1;        // 0=stopped, 1=fault, 10=position
    float position = NAN; // revolutions
    float velocity = NAN; // rev/s
    float torque = NAN;   // N*m
    int voltage_dV = -1;  // 0.5 V units
    int temp_C = -1;
    int fault = -1;
    uint32_t replies = 0;
  };
  const Status& status() const { return m_status; }

  // ---- MoteusN1Base diagnostic overrides (callable through a base pointer) ----
  bool parseReplyMsg(const CANFD_message_t& rx) override { return parseReply(rx); }
  void sendQueryMsg() override { sendQuery(); }
  int statusMode() override { return m_status.mode; }
  int statusFault() override { return m_status.fault; }
  float statusPosition() override { return m_status.position; }

  /**
   * @brief Send a query (reply requested) using the standard mjbots default
   * query format: mode(int8), position/velocity/torque(float),
   * voltage/temperature/fault(int8).
   */
  void sendQuery() {
    CANFD_message_t msg;
    initFrame(msg);
    msg.id = 0x8000 | (uint32_t)m_canId;  // reply-required bit
    msg.buf[0] = 0x11;  // read int8 x1
    msg.buf[1] = 0x00;  //   reg 0x000 (Mode)
    msg.buf[2] = 0x1F;  // read float x3
    msg.buf[3] = 0x01;  //   regs 0x001-0x003 (Position, Velocity, Torque)
    msg.buf[4] = 0x13;  // read int8 x3
    msg.buf[5] = 0x0D;  //   regs 0x00d-0x00f (Voltage, Temperature, Fault)
    msg.len = 6;
    m_can.write(msg);
  }

  /**
   * @brief Parse a received frame for this node's reply (multiplex protocol).
   * Reply arbitration ID has this node's ID in bits 8-14 (source field).
   * Handles reply subframes (0x20-0x2f) at int8/int16/int32/float resolutions
   * with the standard mjbots register scalings.
   * @return true if the frame was a reply from this node
   */
  bool parseReply(const CANFD_message_t& rx) {
    if (((rx.id >> 8) & 0x7F) != (uint32_t)m_canId) return false;
    m_status.replies++;
    uint8_t i = 0;
    while (i < rx.len) {
      uint8_t op = rx.buf[i];
      if (op == 0x50) { i++; continue; }        // NOP padding
      if ((op & 0xF0) != 0x20) break;           // only handle reply ops
      uint8_t res = (op >> 2) & 0x3;            // 0=int8,1=int16,2=int32,3=float
      uint8_t count = op & 0x3;
      i++;
      if (count == 0) {                          // long form: count byte follows
        if (i >= rx.len) break;
        count = rx.buf[i++];
      }
      if (i >= rx.len) break;
      uint16_t reg = rx.buf[i++];               // varuint (regs < 128 are 1 byte)
      static const uint8_t kSize[4] = {1, 2, 4, 4};
      for (uint8_t c = 0; c < count && i + kSize[res] <= rx.len; c++, reg++) {
        float v = NAN;
        int32_t raw = 0;
        if (res == 0) { raw = (int8_t)rx.buf[i]; v = raw; }
        else if (res == 1) { int16_t r16; memcpy(&r16, &rx.buf[i], 2); raw = r16; v = r16; }
        else if (res == 2) { memcpy(&raw, &rx.buf[i], 4); v = raw; }
        else { memcpy(&v, &rx.buf[i], 4); raw = (int32_t)v; }
        i += kSize[res];
        // Apply standard mjbots scalings for the registers we care about
        switch (reg) {
          case 0x000: m_status.mode = raw; break;
          case 0x001: m_status.position = (res == 3) ? v : v * ((res == 1) ? 0.0001f : (res == 2) ? 0.00001f : 0.01f); break;
          case 0x002: m_status.velocity = (res == 3) ? v : v * ((res == 1) ? 0.00025f : (res == 2) ? 0.00001f : 0.1f); break;
          case 0x003: m_status.torque = (res == 3) ? v : v * ((res == 1) ? 0.01f : (res == 2) ? 0.001f : 0.5f); break;
          case 0x00d: m_status.voltage_dV = raw; break;
          case 0x00e: m_status.temp_C = raw; break;
          case 0x00f: m_status.fault = raw; break;
          default: break;
        }
      }
    }
    return true;
  }

public:
  int canId() const override { return m_canId; }
  /// Change the target CAN ID at runtime (bench-test convenience).
  void setCanId(int id) { m_canId = id; }

private:
  FlexCAN_T4FD<_bus, RX_SIZE_256, TX_SIZE_16>& m_can;
  int m_canId;
  bool m_printOnWrite;
  float m_lastPosition;
  float m_lastVelocity;
  uint8_t m_lastError;
  Status m_status;

  // Moteus register protocol constants (see mjbots moteus reference docs)
  static constexpr uint8_t kWriteInt8x1 = 0x01;   // write 1x int8 register
  static constexpr uint8_t kWriteFloatx2 = 0x0e;  // write 2x float registers (0x0c | 2)
  static constexpr uint8_t kRegMode = 0x00;       // register 0x000: Mode
  static constexpr uint8_t kRegCmdPosition = 0x20;// register 0x020: Command Position
  static constexpr uint8_t kModeStopped = 0;      // Mode::kStopped
  static constexpr uint8_t kModePosition = 10;    // Mode::kPosition
  static constexpr uint8_t kNop = 0x50;           // padding byte (Multiplex::kNop)

  /**
   * @brief Initialize a CAN-FD frame with the Moteus arbitration ID.
   * Arbitration ID = destination (low 8 bits) | source (bits 8-15).
   * Reply-required bit (0x8000) not set: fire-and-forget commands.
   * BRS is disabled to match the mjbots Arduino library (verified working
   * with TeensyBasicControl example): whole frame at the 1 Mbps nominal rate.
   */
  void initFrame(CANFD_message_t& msg) {
    msg.id = m_canId;         // dest = canId, source = 0, no reply requested
    msg.flags.extended = 1;   // Moteus reference implementation uses extended IDs
    msg.brs = 0;              // NO bit rate switch (mjbots library disables BRS)
    msg.edl = 1;              // CAN-FD frame
    msg.len = 0;
  }

  /**
   * @brief Send a stop-mode command (register 0x000 Mode = kStopped).
   * Also clears latched faults.
   */
  void sendStop() {
    CANFD_message_t msg;
    initFrame(msg);
    msg.buf[0] = kWriteInt8x1;
    msg.buf[1] = kRegMode;
    msg.buf[2] = kModeStopped;
    msg.len = 3;
    m_can.write(msg);
  }

  /**
   * @brief Send a position-mode command with float position and velocity.
   * Frame layout (Moteus register protocol):
   *   0x01 0x00 0x0a          -> write int8 Mode = kPosition
   *   0x0e 0x20 <f32> <f32>   -> write float CommandPosition, CommandVelocity
   * position=NaN means "hold current position" (pure velocity tracking).
   * @param position Target position in revolutions (or NaN)
   * @param velocity Target velocity in revolutions/sec
   */
  void sendPositionMode(float position, float velocity) {
    CANFD_message_t msg;
    initFrame(msg);
    msg.buf[0] = kWriteInt8x1;
    msg.buf[1] = kRegMode;
    msg.buf[2] = kModePosition;
    msg.buf[3] = kWriteFloatx2;
    msg.buf[4] = kRegCmdPosition;
    memcpy(&msg.buf[5], &position, 4);
    memcpy(&msg.buf[9], &velocity, 4);
    // Pad to the next CAN-FD DLC size (16) with NOPs so the padding
    // bytes the controller receives are valid protocol.
    msg.buf[13] = kNop;
    msg.buf[14] = kNop;
    msg.buf[15] = kNop;
    msg.len = 16;
    m_can.write(msg);
  }
};

/********************************************************************/
#endif // __MOTEUS_N1_H
