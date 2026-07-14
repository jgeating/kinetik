// Shared implementation for the MASTER/SLAVE "spin all motors" hardware tests.
//
// Spins every Moteus N1 motor on the Teensy's CAN3 (CAN-FD) bus at a slow,
// constant velocity to confirm each node is wired, powered, and responding.
// Bring-up / bench test ONLY: bypasses the planner, RC input, pads, and the
// master<->slave serial link.
//
// A test firmware includes this header and defines its motor IDs and a label,
// then calls spinTestSetup()/spinTestLoop() from setup()/loop().
//
// SAFETY: motors WILL rotate continuously. Make sure wheels are off the ground
// / free to spin before running. Lower SPIN_VELOCITY if needed.

#ifndef __SPIN_ALL_COMMON_H
#define __SPIN_ALL_COMMON_H

#include <Arduino.h>
#include <math.h>
#include <FlexCAN_T4.h>
#include "MoteusN1.h"

// Velocity commanded to every motor, in rev/s. Keep small for a bench test.
static const float SPIN_VELOCITY = 0.1f;
// Control tick period (us). Moteus faults if commands stop arriving.
static const unsigned long CONTROL_PERIOD_US = 2000;  // 500 Hz

// CAN-FD requires the FlexCAN_T4FD class; only CAN3 has FD hardware.
static FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> driveCanBus;

// Populated by spinTestSetup() from the caller-supplied ID list.
static MoteusN1<CAN3>* g_motors[8];
static const int* g_motorIds = nullptr;
static int g_numMotors = 0;
static unsigned long g_lastControlUpdate = 0;

static void spinStopAll() {
  for (int i = 0; i < g_numMotors; i++) g_motors[i]->disable();
}

// motorIds: CAN IDs to drive. label: banner text. Call once from setup().
static void spinTestSetup(const int* motorIds, int numMotors, const char* label) {
  g_motorIds = motorIds;
  g_numMotors = numMotors;
  for (int i = 0; i < numMotors; i++) {
    g_motors[i] = new MoteusN1<CAN3>(driveCanBus, motorIds[i]);
  }

  Serial.begin(115200);
  unsigned long serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart) < 2000) {}
  delay(100);
  Serial.print("=== HW TEST: ");
  Serial.print(label);
  Serial.println(" spin-all on CAN3 ===");
  Serial.println("[INFO] Type BOOTLOAD via serial to trigger bootloader mode (no reset needed)");
  Serial.print("[INFO] Spinning all motors at ");
  Serial.print(SPIN_VELOCITY, 3);
  Serial.println(" rev/s. Ensure wheels are free to spin!");

  // CAN3 - CAN-FD, 1 Mbps arbitration, BRS off in frames (matches penny_v4).
  driveCanBus.begin();
  driveCanBus.setRegions(64);  // 64-byte mailboxes; default 8-byte regions truncate FD frames
  driveCanBus.setBaudRate(CAN_1M_6M);
  Serial.println("[CAN3] CAN-FD initialized: 1 Mbps, BRS off");

  // Scan IDs 0-23 to report which Moteus nodes actually reply on this bus.
  Serial.print("[SCAN] replies from:");
  for (int id = 0; id <= 23; id++) {
    g_motors[0]->setCanId(id);
    g_motors[0]->sendQuery();
    delay(5);
    bool replied = false;
    CANFD_message_t rx;
    while (driveCanBus.read(rx)) {
      if (((rx.id >> 8) & 0x7F) == (uint32_t)id) replied = true;
    }
    if (replied) { Serial.print(' '); Serial.print(id); }
  }
  Serial.println();
  g_motors[0]->setCanId(motorIds[0]);  // restore expected ID after scan

  // Clear any latched faults so velocity commands are accepted.
  Serial.println("Clearing faults on all motors...");
  for (int i = 0; i < numMotors; i++) {
    g_motors[i]->enable();  // sends stop, which clears latched faults
    delay(10);
  }
  Serial.println("Setup complete. Spinning...");
}

// Call every iteration from loop().
static void spinTestLoop() {
  // Allow uploading without pressing the reset button.
  if (Serial.available() >= 8) {  // "BOOTLOAD" is 8 chars
    String cmd = Serial.readStringUntil('\n');
    if (cmd == "BOOTLOAD") {
      Serial.println("[INFO] Rebooting into bootloader...");
      spinStopAll();
      delay(100);
      _reboot_Teensyduino_();
    }
  }

  unsigned long now = micros();
  if (now - g_lastControlUpdate >= CONTROL_PERIOD_US) {
    g_lastControlUpdate = now;
    for (int i = 0; i < g_numMotors; i++) g_motors[i]->setVelocity(SPIN_VELOCITY);
  }

  // Drain replies so the RX FIFO does not overflow and update per-node status.
  CANFD_message_t rx;
  while (driveCanBus.read(rx)) {
    for (int i = 0; i < g_numMotors; i++) g_motors[i]->parseReply(rx);
  }

  // 1 Hz heartbeat: CAN error counters + per-node reply totals.
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat >= 1000) {
    lastHeartbeat = millis();
    uint32_t esr1 = *(volatile uint32_t*)0x401D8020;  // CAN3 ESR1
    uint32_t ecr = *(volatile uint32_t*)0x401D801C;   // CAN3 ECR (TEC/REC)
    Serial.print("[HB] synch=");
    Serial.print((esr1 >> 18) & 0x1);
    Serial.print(" TEC=");
    Serial.print(ecr & 0xFF);
    Serial.print(" REC=");
    Serial.print((ecr >> 8) & 0xFF);
    Serial.print(" replies:");
    for (int i = 0; i < g_numMotors; i++) {
      g_motors[i]->sendQuery();  // re-query so we observe live replies
      Serial.print(" id");
      Serial.print(g_motorIds[i]);
      Serial.print('=');
      Serial.print(g_motors[i]->status().replies);
    }
    Serial.println();
  }
}

#endif  // __SPIN_ALL_COMMON_H
