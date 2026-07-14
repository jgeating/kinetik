// Penny V4 SLAVE Teensy 4.1
//
// Controls two swerve modules (4x Moteus N1 on CAN3, CAN-FD):
//   Front Right: steer ID 10, drive ID 20 | Front Left: steer ID 3, drive ID 13
// Receives commands from the master Teensy over Serial7 (RX7 = pin 28)
// and forwards them to CAN as fast as possible on packet receipt.
// See include/TeensyLink.h for the wire protocol.
//
// Build/upload: pio run -e penny_v4_slave -t upload

#include <Arduino.h>
#include <math.h>
#include <FlexCAN_T4.h>
#include "MoteusN1.h"
#include "TeensyLink.h"

// CAN Bus setup (same proven init as master: FD on CAN3 only)
FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> driveCanBus;

// Front modules commanded by the master over Serial7:
//   slot 0 = Front Right (steer 10, drive 20)  [master module index 0]
//   slot 1 = Front Left  (steer 3,  drive 13)  [master module index 3]
// Note: Front Right drive uses ID 20 (ID 0 was unusable, so that module was set to 20).
MoteusN1<CAN3, 10> frontRightSteer(driveCanBus, 10);
MoteusN1<CAN3, 3> frontLeftSteer(driveCanBus, 3);
MoteusN1<CAN3, 20> frontRightDrive(driveCanBus, 20);
MoteusN1<CAN3, 13> frontLeftDrive(driveCanBus, 13);

MoteusN1Base* steerMotors[2] = {&frontRightSteer, &frontLeftSteer};
MoteusN1Base* driveMotors[2] = {&frontRightDrive, &frontLeftDrive};

// Link parser state
static uint8_t rxBuf[sizeof(TeensyLinkPacket)];
static size_t rxIdx = 0;

// Link statistics / watchdog
static unsigned long lastPacketMs = 0;
static uint32_t packetCount = 0;
static uint32_t checksumErrors = 0;
static bool motorsStopped = true;

void stopAllMotors() {
  for (int i = 0; i < 2; i++) {
    steerMotors[i]->disable();
    driveMotors[i]->disable();
  }
  motorsStopped = true;
}

// TEMP debug: last commanded values echoed on the heartbeat
static float lastCmdSteer[2] = {NAN, NAN};
static float lastCmdDrive[2] = {0.0f, 0.0f};

void applyPacket(const TeensyLinkPacket& pkt) {
  if (!(pkt.flags & TEENSY_LINK_FLAG_ENABLE)) {
    if (!motorsStopped) stopAllMotors();
    return;
  }
  motorsStopped = false;
  // Forward to CAN immediately: 4 writes, no buffering
  for (int i = 0; i < 2; i++) {
    lastCmdSteer[i] = pkt.steerPos[i];
    lastCmdDrive[i] = pkt.driveVel[i];
    if (isnan(pkt.steerPos[i])) {
      // Turning disabled by master: HOLD position (velocity 0) so the steer
      // controller stays commanded every tick and does not fall into the
      // Moteus position-command timeout (kPositionTimeout), which would make
      // it refuse position commands when turning is re-enabled.
      steerMotors[i]->setVelocity(0.0f);
    } else {
      steerMotors[i]->setPosition(pkt.steerPos[i]);
    }
    driveMotors[i]->setVelocity(pkt.driveVel[i]);
  }
}

/// Byte-wise parser: hunts for sync, validates checksum, resyncs on garbage.
void pollLink() {
  while (Serial7.available()) {
    uint8_t b = Serial7.read();
    if (rxIdx == 0) {
      if (b == TEENSY_LINK_SYNC1) rxBuf[rxIdx++] = b;
    } else if (rxIdx == 1) {
      if (b == TEENSY_LINK_SYNC2) {
        rxBuf[rxIdx++] = b;
      } else {
        rxIdx = (b == TEENSY_LINK_SYNC1) ? 1 : 0;  // allow AA AA 55
      }
    } else {
      rxBuf[rxIdx++] = b;
      if (rxIdx >= sizeof(TeensyLinkPacket)) {
        rxIdx = 0;
        TeensyLinkPacket pkt;
        memcpy(&pkt, rxBuf, sizeof(pkt));
        if (pkt.checksum == teensyLinkChecksum(pkt)) {
          packetCount++;
          lastPacketMs = millis();
          applyPacket(pkt);
        } else {
          checksumErrors++;
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  unsigned long serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart) < 2000) {}
  delay(100);
  Serial.println("Penny V4 SLAVE - Moteus N1 front modules: FR steer 10/drive 20, FL steer 3/drive 13 (CAN3)"); 
  Serial.println("[INFO] Type BOOTLOAD via serial to trigger bootloader mode (no reset needed)");

  // CAN3 - CAN-FD, 1 Mbps arbitration, BRS off in frames (matches master)
  driveCanBus.begin();
  driveCanBus.setRegions(64);  // 64-byte mailboxes; default 8-byte regions truncate FD frames
  driveCanBus.setBaudRate(CAN_1M_6M);
  Serial.println("[CAN3] CAN-FD initialized: 1 Mbps, BRS off");

  // Master link on Serial7 (RX7 = pin 28)
  Serial7.begin(TEENSY_LINK_BAUD);
  Serial.println("[LINK] Serial7 listening at " + String(TEENSY_LINK_BAUD) + " baud");

  // Send stop to clear any latched faults so position commands are accepted
  Serial.println("Clearing faults on 4 Moteus N1 motors...");
  for (int i = 0; i < 2; i++) {
    steerMotors[i]->enable();
    delay(10);
    driveMotors[i]->enable();
    delay(10);
  }

  Serial.println("Setup complete. Waiting for master commands.");
}

void loop() {
  // Check for bootload command on USB serial (allows upload without reset)
  if (Serial.available() >= 8) {  // "BOOTLOAD" is 8 chars
    String cmd = Serial.readStringUntil('\n');
    if (cmd == "BOOTLOAD") {
      Serial.println("[INFO] Rebooting into bootloader...");
      delay(100);
      _reboot_Teensyduino_();
    }
  }

// Forward master commands to CAN with minimum latency
  pollLink();

  // Link watchdog: stop motors if the master goes quiet
  if (!motorsStopped && millis() - lastPacketMs > TEENSY_LINK_TIMEOUT_MS) {
    stopAllMotors();
    Serial.println("[LINK] Timeout - motors stopped");
  }

  // Heartbeat: link statistics once per second
  static unsigned long lastHeartbeat = 0;
  static uint32_t lastPacketCount = 0;
  if (millis() - lastHeartbeat >= 1000) {
    lastHeartbeat = millis();
    Serial.print("[HB] link=");
    Serial.print(millis() - lastPacketMs <= TEENSY_LINK_TIMEOUT_MS ? "OK" : "DOWN");
    Serial.print(" rate=");
    Serial.print(packetCount - lastPacketCount);
    Serial.print("/s crcErr=");
    Serial.print(checksumErrors);
    Serial.print(" motors=");
    Serial.print(motorsStopped ? "STOPPED" : "active");
    // TEMP debug: echo last commanded steer/drive per slot (FR=0, FL=1)
    Serial.print(" | FR[s=");
    Serial.print(lastCmdSteer[0], 3);
    Serial.print(" d=");
    Serial.print(lastCmdDrive[0], 2);
    Serial.print("] FL[s=");
    Serial.print(lastCmdSteer[1], 3);
    Serial.print(" d=");
    Serial.print(lastCmdDrive[1], 2);
    Serial.println("]");
    lastPacketCount = packetCount;
  }
}
