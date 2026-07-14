#ifndef __TEENSY_LINK_H
#define __TEENSY_LINK_H

#include <stdint.h>
#include <stddef.h>

/**
 * Serial link between master and slave Teensy 4.1 (Serial7: RX7=pin 28,
 * TX7=pin 29, wired crossed: master TX7 -> slave RX7, common GND).
 *
 * Master -> slave packet carrying commands for the slave's two swerve
 * modules. Slot 0: steer ID 0, drive ID 10 (Back Right).
 *          Slot 1: steer ID 3, drive ID 13 (Front Left).
 *
 * Steering offsets are applied by the MASTER before sending, so all
 * calibration lives in the master firmware.
 *
 * NaN in steerPos/driveVel means "don't command" (MoteusN1 ignores NaN).
 * With the ENABLE flag clear, the slave stops all motors (values ignored).
 * The slave also stops all motors if no valid packet arrives for
 * TEENSY_LINK_TIMEOUT_MS (link watchdog).
 */

static const uint32_t TEENSY_LINK_BAUD = 2000000;
static const uint8_t TEENSY_LINK_SYNC1 = 0xAA;
static const uint8_t TEENSY_LINK_SYNC2 = 0x55;
static const uint8_t TEENSY_LINK_FLAG_ENABLE = 0x01;
static const uint32_t TEENSY_LINK_TIMEOUT_MS = 100;

struct __attribute__((packed)) TeensyLinkPacket {
  uint8_t sync1;      // TEENSY_LINK_SYNC1
  uint8_t sync2;      // TEENSY_LINK_SYNC2
  uint8_t flags;      // TEENSY_LINK_FLAG_*
  float steerPos[2];  // revolutions (offset already applied by master)
  float driveVel[2];  // rev/s
  uint8_t checksum;   // XOR of flags + all float bytes
};

/// XOR checksum over payload (flags through last float).
inline uint8_t teensyLinkChecksum(const TeensyLinkPacket& p) {
  const uint8_t* b = (const uint8_t*)&p.flags;
  uint8_t c = 0;
  for (size_t i = 0; i < 1 + sizeof(p.steerPos) + sizeof(p.driveVel); i++) {
    c ^= b[i];
  }
  return c;
}

#endif  // __TEENSY_LINK_H
