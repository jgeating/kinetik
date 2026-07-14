#ifndef __MOTEUS_FLEXCAN_H
#define __MOTEUS_FLEXCAN_H

#include <FlexCAN_T4.h>
#include <Moteus.h>

/**
 * @brief FlexCAN_T4 adapter for mjbots Moteus library.
 * 
 * The official Moteus library (v1.1.1) expects ACAN_T4, but this project
 * uses FlexCAN_T4. This adapter bridges the two APIs to allow using
 * FlexCAN_T4 buses with MoteusController.
 * 
 * @tparam _bus CAN bus type (CAN1, CAN3, etc.)
 */
template<CAN_DEV_TABLE _bus>
class MoteusFlexCAN {
public:
  /**
   * @brief Initialize the adapter with a FlexCAN_T4 bus reference.
   * @param bus Reference to the FlexCAN_T4 bus instance
   */
  MoteusFlexCAN(FlexCAN_T4<_bus, RX_SIZE_256, TX_SIZE_16>& bus)
      : bus_(bus) {}

  /**
   * @brief Poll the bus for new messages (no-op for FlexCAN_T4, which is interrupt-driven).
   */
  void poll() {
    // FlexCAN_T4 is interrupt-driven; no polling needed
  }

  /**
   * @brief Check if a CAN-FD message is available.
   * @return True if a message is waiting
   */
  bool available() {
    return bus_.getRXQueueCount() > 0;
  }

  /**
   * @brief Receive a CAN-FD message from the bus.
   * @param msg Output: the CAN-FD message received
   * @return True if a message was successfully received
   */
  bool receive(CANFDMessage& msg) {
    CAN_message_t rx_msg;
    if (bus_.read(rx_msg)) {
      msg.id = rx_msg.id;
      msg.ext = rx_msg.flags.extended;
      msg.len = rx_msg.len;
      memcpy(msg.data, rx_msg.buf, rx_msg.len);
      return true;
    }
    return false;
  }

  /**
   * @brief Send a CAN-FD message on the bus.
   * @param msg The CAN-FD message to send
   * @return True if the message was successfully queued for transmission
   */
  bool tryToSend(const CANFDMessage& msg) {
    CAN_message_t tx_msg;
    tx_msg.id = msg.id;
    tx_msg.flags.extended = msg.ext;
    tx_msg.len = msg.len;
    memcpy(tx_msg.buf, msg.data, msg.len);
    return bus_.write(tx_msg) >= 0;  // write returns -1 on error, >= 0 on success
  }

  // Alias for tryToSendReturnStatusFD (expected by some Moteus versions)
  bool tryToSendReturnStatusFD(const CANFDMessage& msg) {
    return tryToSend(msg);
  }

private:
  FlexCAN_T4<_bus, RX_SIZE_256, TX_SIZE_16>& bus_;
};

#endif // __MOTEUS_FLEXCAN_H
