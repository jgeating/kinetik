/**
 * @file SbusReceiver.h
 * @brief Platform-agnostic SBUS receiver interface
 *
 * This header defines the interface for SBUS receivers.
 * Implementation details are handled by platform-specific .cpp files:
 * - SbusReceiver.cpp: Real hardware implementation (uses sbus.h)
 * - SbusReceiver_sim.cpp: Simulation implementation (uses HAL)
 */

#ifndef _SBUSRECEIVER_
#define _SBUSRECEIVER_

#include <cstdint>

/**
 * @brief SBUS receiver channel enumeration
 */
enum class SbusReceiverChannels : int8_t
{
  LEFT_STICK_VERT = 0,
  LEFT_STICK_HOR = 3,
  RIGHT_STICK_VERT = 2,
  RIGHT_STICK_HOR = 1,
  LEFT_KNOB = 4,
  RIGHT_KNOB = 5,
  RED_SWITCH = 6,
  BLUE_SWITCH = 7,
};

/**
 * @brief Platform-agnostic SBUS receiver interface
 *
 * This class provides a consistent interface for SBUS receivers
 * across different platforms (real hardware vs simulation).
 */
class SbusReceiver
{
public:
  /**
   * @brief Constructor
   */
  SbusReceiver();

  /**
   * @brief Initialize the SBUS receiver
   */
  void init();

  /**
   * @brief Read data from the SBUS receiver
   */
  void read();

  /**
   * @brief Check if RC signal is lost
   * @return 1 if signal is lost, 0 if signal is good
   */
  int rcLost();

  /**
   * @brief Get channel data with default value
   * @param channel Channel to read
   * @param defaultValue Default value if channel unavailable
   * @return Channel value normalized to [-1.0, 1.0] range
   */
  double getChannelData(SbusReceiverChannels channel, double defaultValue = 0.0);

  /**
   * @brief Get blue switch position
   * @return Switch value normalized to [-1.0, 1.0] range
   */
  double getBlueSwitch();

  /**
   * @brief Check if blue switch is in up position
   * @return true if switch is up
   */
  bool isBlueSwitchUp();

  /**
   * @brief Check if blue switch is in down position
   * @return true if switch is down
   */
  bool isBlueSwitchDown();

  /**
   * @brief Check if blue switch is in center position
   * @return true if switch is centered
   */
  bool isBlueSwitchCentered();

  /**
   * @brief Get handheld remote value
   * @return Remote value normalized to [-1.0, 1.0] range
   */
  double getHandheld();

  /**
   * @brief Get red switch position
   * @return Switch value normalized to [-1.0, 1.0] range
   */
  double getRedSwitch();

  /**
   * @brief Get right knob position
   * @return Knob value normalized to [-1.0, 1.0] range
   */
  double getRightKnob();

  /**
   * @brief Get left stick vertical position
   * @return Stick value normalized to [-1.0, 1.0] range
   */
  double getLeftVert();

  /**
   * @brief Get left stick horizontal position
   * @return Stick value normalized to [-1.0, 1.0] range
   */
  double getLeftHor();

  /**
   * @brief Get right stick vertical position
   * @return Stick value normalized to [-1.0, 1.0] range
   */
  double getRightVert();

  /**
   * @brief Get right stick horizontal position
   * @return Stick value normalized to [-1.0, 1.0] range
   */
  double getRightHor();

private:
  // Platform-specific implementation details are hidden
  // Real implementation will have bfs::SbusRx, bfs::SbusData, etc.
  // Simulation implementation will have different internal state
  // This is handled by the implementation files, not the interface
};

#endif // _SBUSRECEIVER_