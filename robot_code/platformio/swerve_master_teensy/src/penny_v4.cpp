#include <Arduino.h>
#include <math.h>
#include <FlexCAN_T4.h>
#include "Constants.h"
#include "MoteusN1.h"
#include "SbusReceiver.h"
#include "shared/utils.h"
#include "SlewRateLimiter.h"
#include "ClosestAngleSteering.h"
#include "Planner.h"
#include "TelemetrySerial.h"
#include "SwerveTelemetry.h"
#include "Pads.h"
#include "TeensyLink.h"

#pragma #region
// ===== MOTOR CONFIGURATION (MASTER Teensy) =====
// All 8 motors are Moteus N1 (CAN-FD), split across two Teensy 4.1s because
// each Teensy has only one CAN-FD bus (CAN3).
//   MASTER (this firmware): Back Right + Back Left - steer IDs 1, 2 | drive IDs 11, 12
//   SLAVE (penny_v4_slave): Front Right + Front Left - steer IDs 10, 3 | drive IDs 20, 13
// Remote-module commands are forwarded to the slave over Serial7 (TX7 = pin 29)
// once per control tick. See include/TeensyLink.h for the protocol.

// Frequently used toggles etc. 
bool enable_serial_telemetry = false; 

// TEST MODE: bench test of a single Moteus N1 on CAN3 (auto-detects node ID).
// Skips ODrive/steering init and full swerve control. Set to false to
// restore normal operation.
const bool TEST_MODE_SINGLE_MOTEUS = false;

// DIAGNOSTIC: internal CAN3 loopback self-test at boot. Loopback routes TX->RX
// inside the chip (no transceiver/bus needed) and self-syncs, so it isolates
// the Teensy CAN peripheral + firmware from external wiring. Leave false for
// normal operation.
const bool CAN_LOOPBACK_SELFTEST = true;

// Function declarations
void controlSwerveModules();
void stopAllMotors();
void print_load_cell_values();
void print_pads_outputs();
void print_pad_preload();

// CAN Bus setup
// NOTE: CAN-FD frames require the FlexCAN_T4FD class (the classic FlexCAN_T4
// class silently drops CANFD_message_t writes). Only CAN3 has FD hardware.
FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> driveCanBus;  // CAN3: 4x Moteus N1 (local modules)

// Swerve Kinematics structs 
SwerveTrajectory traj;
pad_vars padVars;
SwerveKinematics kin;

// Swerve module: local (motors on this Teensy's CAN3) or remote (commands
// forwarded to the slave Teensy over Serial7; motor pointers are null).
struct SwerveModule {
  MoteusN1Base* steerMotor;   // null for remote modules
  MoteusN1Base* driveMotor;   // null for remote modules
  String name;
  double steeringOffset;
  int remoteSlot;             // -1 = local; 0/1 = slot in TeensyLinkPacket

  SwerveModule(MoteusN1Base* steer, MoteusN1Base* drive, const String& moduleName,
               double offset, int slot = -1)
    : steerMotor(steer), driveMotor(drive), name(moduleName),
      steeringOffset(offset), remoteSlot(slot) {}
};

// ===== MOTOR INSTANCES (local, CAN3) =====
// (CAN ID is a runtime constructor arg; single template type so the motors
// can share an array with the full query/status API.)
// Master controls the two BACK modules (indices 1 and 2):
MoteusN1<CAN3> backRightSteer(driveCanBus, 1);       // index 1 Back Right steering (ID 1)
MoteusN1<CAN3> backLeftSteer(driveCanBus, 2);        // index 2 Back Left steering (ID 2)
MoteusN1<CAN3> backRightDrive(driveCanBus, 11);      // index 1 Back Right drive (ID 11)
MoteusN1<CAN3> backLeftDrive(driveCanBus, 12);       // index 2 Back Left drive (ID 12)

// All local motors, for status queries and bring-up tests
MoteusN1<CAN3>* localMotors[4] = {
  &backRightSteer, &backLeftSteer, &backRightDrive, &backLeftDrive
};

// TEST: bench-test Moteus N1 node on CAN3 (ID auto-detected at boot)
MoteusN1<CAN3> testMotor(driveCanBus, 1);

// Create array of all swerve modules
const size_t NUM_MODULES = 4;

// Module index -> physical corner is FIXED by the kinematics (angle = 90*i - 45):
//   [0] Front Right (remote, slave slot 0): steer 10, drive 20
//   [1] Back Right  (local):                steer 1,  drive 11
//   [2] Back Left   (local):                steer 2,  drive 12
//   [3] Front Left  (remote, slave slot 1): steer 3,  drive 13
// Offsets applied here on the master. NOTE: offsets below were measured under
// the OLD (scrambled) mapping and must be RE-MEASURED now that indexing is fixed.
SwerveModule allModules[NUM_MODULES] = {
  SwerveModule(nullptr, nullptr, "Front Right",  7 * M_PI / 180.0, 0),               // [0] remote slot 0
  SwerveModule(&backRightSteer, &backRightDrive, "Back Right", 89 * M_PI / 180.0),   // [1] local
  SwerveModule(&backLeftSteer,  &backLeftDrive,  "Back Left",  29 * M_PI / 180.0),   // [2] local
  SwerveModule(nullptr, nullptr, "Front Left",  28 * M_PI / 180.0, 1),               // [3] remote slot 1
};

// Outgoing command packet for the slave (front modules), sent once per control tick
TeensyLinkPacket slavePacket;

// Timing
unsigned long lastControlUpdate = 0;
const unsigned long CONTROL_PERIOD = 2000;
  
SbusReceiver sbusReceiver;
SlewRateLimiter steerLimiter(PI * 4);
SlewRateLimiter driveLimiter(20);
ClosestAngleSteering closestAngleSteering;
Planner planner((double)CONTROL_PERIOD, traj, padVars, kin);
TelemetrySerial telemetry;
Pads pads;

double stickAngle = 0.0;
const double STEERING_DEADZONE = 0.25;
const double MAX_VELOCITY = 10; // m/s - adjust based on your robot's capabilities



#pragma endregion

void setup() {
  Serial.begin(115200);
  // Wait briefly for the USB host to open the serial port so early prints aren't lost
  unsigned long serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart) < 2000) {
    // wait up to 2s for host connection
  }
  delay(100);
  Serial.println("Penny V4 MASTER - Moteus N1 steer 1, 2 + drive 11, 12 (CAN3)");
  Serial.println("[INFO] Type BOOTLOAD via serial to trigger bootloader mode (no reset needed)");
  if (TEST_MODE_SINGLE_MOTEUS) {
    Serial.println("[TEST MODE] Single Moteus N1 on CAN3. Steering/swerve disabled.");
  }

  // CAN3 (driveCanBus) - 4x Moteus N1 (local modules), CAN-FD
  // 1 Mbps arbitration; BRS disabled in frames (matches mjbots Arduino
  // library / TeensyBasicControl, which runs the whole frame at 1 Mbps)
  driveCanBus.begin();
  driveCanBus.setRegions(64);  // 64-byte mailboxes; default 8-byte regions truncate FD frames
  driveCanBus.setBaudRate(CAN_1M_6M);  // nominal 1 Mbps; data rate unused while BRS=0
  Serial.println("[CAN3] CAN-FD initialized: 1 Mbps, BRS off");

  if (CAN_LOOPBACK_SELFTEST) {
    // Enable internal loopback (MCR.LPB=bit0) + self-reception (clear SRXDIS=bit17)
    // in freeze mode, then send one frame and confirm it loops back to RX.
    // Proves the CAN peripheral + FlexCAN driver work independent of wiring.
    Serial.println("[SELFTEST] CAN3 internal loopback...");
    volatile uint32_t* mcr = (volatile uint32_t*)0x401D8000;  // CAN3 MCR
    *mcr |= (1UL << 30) | (1UL << 28);            // FRZ | HALT
    while (!(*mcr & (1UL << 24))) {}              // wait FRZ_ACK
    *mcr &= ~(1UL << 17);                         // clear SRXDIS (self-reception on)
    volatile uint32_t* ctrl1 = (volatile uint32_t*)0x401D8004;
    *ctrl1 |= (1UL << 12);                        // LPB (loopback) - CTRL1 bit 12
    *ctrl1 &= ~(1UL << 3);                         // clear LOM (listen-only) so TX drives
    *mcr &= ~(1UL << 28);                         // clear HALT
    while (*mcr & (1UL << 24)) {}                 // wait !FRZ_ACK

    CANFD_message_t tx;
    tx.id = 0x123; tx.flags.extended = 0; tx.brs = 0; tx.edl = 1; tx.len = 8;
    for (int i = 0; i < 8; i++) tx.buf[i] = 0xA0 + i;
    int wr = driveCanBus.write(tx);
    // Poll for the looped-back frame for up to 50 ms
    CANFD_message_t rx;
    int got = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < 50) {
      if (driveCanBus.read(rx)) { got = 1; break; }
    }
    uint32_t esr1 = *(volatile uint32_t*)0x401D8020;
    Serial.print("[SELFTEST] wr=");
    Serial.print(wr);
    Serial.print(" synch=");
    Serial.print((esr1 >> 18) & 0x1);
    Serial.print(" rx=");
    Serial.print(got);
    if (got) {
      Serial.print(" id=0x");
      Serial.print(rx.id, HEX);
      Serial.print(rx.id == 0x123 ? " -> PERIPHERAL OK (fault is external wiring)"
                                  : " -> unexpected id");
    } else {
      Serial.print(" -> NO LOOPBACK (peripheral/driver setup problem)");
    }
    Serial.println();
    // Restore normal mode: clear LPB, restore SRXDIS, re-enter/exit freeze
    *mcr |= (1UL << 30) | (1UL << 28);
    while (!(*mcr & (1UL << 24))) {}
    *ctrl1 &= ~(1UL << 12);
    *mcr |= (1UL << 17);                          // restore SRXDIS (no self-reception)
    *mcr &= ~(1UL << 28);
    while (*mcr & (1UL << 24)) {}
  }

  // Slave Teensy link on Serial7 (TX7 = pin 29): front-module commands
  Serial7.begin(TEENSY_LINK_BAUD);
  slavePacket.sync1 = TEENSY_LINK_SYNC1;
  slavePacket.sync2 = TEENSY_LINK_SYNC2;
  Serial.println("[LINK] Serial7 to slave at " + String(TEENSY_LINK_BAUD) + " baud");
  delay(100);

  // Onboard LED heartbeat (USB-independent liveness indicator)
  pinMode(13, OUTPUT);

  // Initialize SBUS receiver
  sbusReceiver.init();

  // Initialize telemetry
  if (enable_serial_telemetry) {
    Serial.println("Serial telemetry enabled");
    telemetry.start();
  } else {
    Serial.println("Serial telemetry disabled");
  }
  

  // Initialize pads
  analogReadResolution(12); // Set ADC resolution to 12 bits (0-4095)
  pads.calibrate();
  Serial.println("Right preload: " + String(pads.getRawForce(0) + pads.getRawForce(1) + pads.getRawForce(2) + pads.getRawForce(3), 2) + " N");
  Serial.println("Left preload: " + String(pads.getRawForce(4) + pads.getRawForce(5) + pads.getRawForce(6) + pads.getRawForce(7), 2) + " N");

  if (!TEST_MODE_SINGLE_MOTEUS) {
    // Local Moteus N1 motors: send stop to clear any latched faults so
    // subsequent position-mode commands are accepted. (The slave does the
    // same for its front-module motors at its own boot.)
    Serial.println("Clearing faults on local Moteus N1 motors (steer 1, 2 + drive 11, 12)...");
    for (size_t i = 0; i < NUM_MODULES; i++) {
      if (allModules[i].remoteSlot >= 0) continue;  // slave handles its own
      allModules[i].steerMotor->enable();  // sends stop command (clears faults)
      delay(10);
      allModules[i].driveMotor->enable();
      delay(10);
    }
  }

  if (TEST_MODE_SINGLE_MOTEUS) {
    // Scan the bus for the Moteus node: query each ID and listen for a reply.
    // (Any node ACKs any frame, so we must read a reply to confirm the ID.)
    Serial.println("[SCAN] Querying Moteus IDs 1-15 on CAN3...");
    int foundId = -1;
    for (int id = 1; id <= 15; id++) {
      testMotor.setCanId(id);
      testMotor.sendQuery();
      delay(10);
      CANFD_message_t rx;
      while (driveCanBus.read(rx)) {
        int src = (rx.id >> 8) & 0x7F;  // replies come FROM the node (source != 0)
        if (src != 0) foundId = src;
      }
    }
    // if (foundId > 0) {
    if (0){
      Serial.print("[SCAN] Using Moteus ID ");
      Serial.println(foundId);
      testMotor.setCanId(foundId);
    } else {
      Serial.println("[SCAN] No replies! Falling back to ID 2.");
      testMotor.setCanId(1);
    }
  }

  if (TEST_MODE_SINGLE_MOTEUS) {
    // Clear faults on bench-test node, like moteus1.SetStop() in the example
    Serial.println("Sending stop (fault clear) to test motor...");
    testMotor.enable();
    delay(100);
  } else {
    // Scan IDs 0-15 to see which Moteus nodes actually reply on this bus.
    Serial.println("[SCAN] Querying Moteus IDs 0-15 on CAN3...");
    Serial.print("[SCAN] replies from:");
    for (int id = 1; id <= 15; id++) {
      testMotor.setCanId(id);
      testMotor.sendQuery();
      delay(5);
      bool replied = false;
      CANFD_message_t rx;
      while (driveCanBus.read(rx)) {
        if (((rx.id >> 8) & 0x7F) == (uint32_t)id) replied = true;
      }
      if (replied) { Serial.print(' '); Serial.print(id); }
    }
    Serial.println();
  }

  Serial.println("Setup complete. Ready for control.");
  Serial.println("Left stick controls swerve modules:");
  Serial.println("- Stick direction sets steering angle");
  Serial.println("- Stick magnitude sets drive velocity");
}

void loop() {
  // Onboard LED heartbeat: ~2 Hz blink proves the loop is running even when
  // USB is unplugged (Serial prints are discarded when disconnected).
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink >= 250) {
    lastBlink = millis();
    digitalWriteFast(13, !digitalReadFast(13));
  }

  // Read SBUS data
  sbusReceiver.read();

  // Check for bootload command on serial (allows upload without reset)
  if (Serial.available() >= 8) {  // "BOOTLOAD" is 8 chars
    String cmd = Serial.readStringUntil('\n');
    if (cmd == "BOOTLOAD") {
      Serial.println("[INFO] Rebooting into bootloader...");
      delay(100);
      _reboot_Teensyduino_();
    }
  }

  // If Ethernet isn't up yet (e.g. DHCP lost the race with link negotiation), retry.
  // telemetry.retryIfNeeded();

  // Drain CAN3 RX: route status replies to the matching local motor
  CANFD_message_t rx;
  while (driveCanBus.read(rx)) {
    if (TEST_MODE_SINGLE_MOTEUS) {
      testMotor.parseReply(rx);
    } else {
      for (auto* m : localMotors) {
        if (m->parseReply(rx)) break;
      }
    }
  }

  // Heartbeat: report Moteus status once per second
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat >= 1000) {
    lastHeartbeat = millis();

    if (TEST_MODE_SINGLE_MOTEUS) {
      const auto& st = testMotor.status();
      Serial.print("[HB] mode=");
      Serial.print(st.mode);               // 10 = position, 0 = stopped, 1 = fault
      Serial.print(" vel=");
      Serial.print(st.velocity, 3);
      Serial.print(" volt=");
      Serial.print(st.voltage_dV / 2.0f, 1);
      Serial.print(" fault=");
      Serial.println(st.fault);
      testMotor.sendQuery();               // ask for fresh status (reply next cycle)
    } else {
      // CAN3 diagnostics: TEC=0 + synch=1 + wr=1 means nodes are ACKing.
      // synch=0 -> controller can't see bus idle (transceiver/RX problem);
      // wr=0 -> TX mailboxes jammed (frames never complete).
      uint32_t ecr = *(volatile uint32_t*)0x401D801C;
      uint32_t esr1 = *(volatile uint32_t*)0x401D8020;
      CANFD_message_t diag;
      diag.id = 0x7FF; diag.flags.extended = 0; diag.brs = 0; diag.edl = 1; diag.len = 0;
      Serial.print("[HB] TEC=");
      Serial.print(ecr & 0xFF);
      Serial.print(" flt=");
      Serial.print((esr1 >> 4) & 0x3);   // 0=active, 1=passive, 2/3=bus-off
      Serial.print(" synch=");
      Serial.print((esr1 >> 18) & 0x1);
      // ESR1 error bits latch the LAST error type (why frames fail):
      //   BIT1(15) BIT0(14) ACK(17) CRC(13) FRM(12) STF(11)
      // ACK only  -> nobody answering (no powered node / wrong IDs)
      // BIT/STF/CRC/FRM -> termination, CANH/CANL swap, or baud mismatch
      Serial.print(" err[");
      if (esr1 & (1UL << 17)) Serial.print("ACK ");
      if (esr1 & (1UL << 15)) Serial.print("BIT1 ");
      if (esr1 & (1UL << 14)) Serial.print("BIT0 ");
      if (esr1 & (1UL << 13)) Serial.print("CRC ");
      if (esr1 & (1UL << 12)) Serial.print("FRM ");
      if (esr1 & (1UL << 11)) Serial.print("STF ");
      Serial.print("]");
      Serial.print(" wr=");
      Serial.print(driveCanBus.write(diag));
      for (auto* m : localMotors) {
        const auto& st = m->status();
        Serial.print(" id");
        Serial.print(m->canId());
        Serial.print("[mode=");
        Serial.print(st.mode);             // 10 = position, 0 = stopped, 1 = fault
        Serial.print(" pos=");
        Serial.print(st.position, 4);      // revolutions (CAL: offset_deg = 360*pos at wheel-straight)
        Serial.print(" vel=");
        Serial.print(st.velocity, 2);
        Serial.print(" flt=");
        Serial.print(st.fault);
        Serial.print("]");
        m->sendQuery();                    // ask for fresh status (reply next cycle)
      }
      Serial.println();
    }
  }

  unsigned long currentTime = micros();

  // Control loop at XXX Hz
  if (currentTime - lastControlUpdate >= CONTROL_PERIOD) {
    lastControlUpdate = currentTime;

    // Check if RC is connected
    if (sbusReceiver.rcLost()) {
      // Stop all motors if RC is lost
      // stopAllMotors();
      // Serial.println("RC Lost - Motors stopped");

      // Still send telemetry so network diagnostics and status updates
      // can be received even when the RC link is down.
      // return;
    }

    // Control all swerve modules
    if (TEST_MODE_SINGLE_MOTEUS) {
      testMotor.setVelocity(0.1);  // TEST: spin bench node at 0.1 rev/s
    } else {
      // Joystick teleop for all modules (red switch gates enable/disable).
      controlSwerveModules();
    }
    delayMicroseconds(100);

    // Send telemetry data
    if (enable_serial_telemetry) {
      telemetry.send();
    }
  }
}

void controlSwerveModules() {
  #pragma region // Process controller inputs
  double leftStickX = sbusReceiver.getLeftHor();  // -1.0 to 1.0
  double leftStickY = sbusReceiver.getLeftVert(); // -1.0 to 1.0
  double rightStickX = sbusReceiver.getRightHor();  // -1.0 to 1.0
  double rightStickY = sbusReceiver.getRightVert(); // -1.0 to 1.0
  double gain_in = constrain((leftStickY + 1.0) / 2.0, 0.0, 1.0);
  double redSwitch = sbusReceiver.getRedSwitch();
  double blueSwitch = sbusReceiver.getBlueSwitch();
  bool disableTurning = redSwitch < -0.5;
  bool disableDriving = redSwitch < 0.5;
  int mode = 0; 
  if (blueSwitch > 0.5) { // blue stick down tele-op
    mode = 0; 
  } else if (blueSwitch < 0.5 && blueSwitch > -0.5) {  // blue stick middle pads control
    mode = 1;
  } else {  // blue stick up  
    mode = 2;
  }
  double throttle_val = constrain(double(analogRead(24) - 1050) / double(3150 - 1050), 0.0, 1.0); // E-bike throttle to analog read pin 24
  #pragma endregion

  // Apply commands to all modules
  slavePacket.flags = (mode != 2) ? TEENSY_LINK_FLAG_ENABLE : 0;
  for (int i = 0; i < NUM_MODULES; i++) {
    if (mode == 0) {
      planner.plan_teleop(rightStickX, rightStickY, leftStickX, gain_in);
    } else if (mode == 1) {
      pads.calcVector();
      // planner.plan_pads(pads.getX(), pads.getY(), pads.getZ(), gain_in);
      // planner.plan_pads(pads.getX(), 0, 0, gain_in * throttle_val);
      planner.plan_pads(pads.getX(), pads.getY(), pads.getZ(), gain_in);
    } else if (mode == 2){
      Serial.println("Zeroing mode - taring pads");
      pads.tare();
      stopAllMotors();
      planner.eStop();
    } else {
      Serial.println("Mode not recognized. This is probably a bug.");
    }

    if (mode != 2) {
      float driveVel = disableDriving ? 0 : planner.getDriveWheelSpeed(i);
      float steerPos = (planner.getSteerAngle(i) + allModules[i].steeringOffset) / (2 * M_PI); // Moteus wants revolutions, not radians
      if (allModules[i].remoteSlot >= 0) {
        // Remote (front) module: stage command for the slave Teensy.
        // When turning is disabled, send NaN so the slave HOLDS the steer
        // motor (keeps it commanded) rather than letting it fault on the
        // Moteus position-command timeout (kPositionTimeout).
        slavePacket.driveVel[allModules[i].remoteSlot] = driveVel;
        slavePacket.steerPos[allModules[i].remoteSlot] = disableTurning ? NAN : steerPos;
      } else {
        // Local (back) module: command CAN directly
        allModules[i].driveMotor->setVelocity(driveVel);
        delayMicroseconds(100);
        if (!disableTurning) {
          allModules[i].steerMotor->setPosition(steerPos);
        } else {
          // Hold current position (velocity 0). Keeps the controller commanded
          // every tick so it does NOT fall into kPositionTimeout and refuse to
          // steer again when turning is re-enabled.
          allModules[i].steerMotor->setVelocity(0.0f);
        }
        delayMicroseconds(100);
      }
    } else {
      planner.plan_pads(0, 0, 0, 0);
    }
  }

  // Ship front-module commands to the slave Teensy (one packet per tick)
  slavePacket.checksum = teensyLinkChecksum(slavePacket);
  Serial7.write((const uint8_t*)&slavePacket, sizeof(slavePacket));

  #pragma region // Send pad telemetry data
  // Raw load cell forces (8 sensors)
  float rawForces[8];
  for (int i = 0; i < 8; i++) {
    rawForces[i] = pads.getRawForce(i);
    telemetry.sendDouble("swerve/pad_raw_force_" + String(i), rawForces[i]);
  }
  
  // Actual load cell forces (8 sensors, zeroed)
  float forces[8];
  for (int i = 0; i < 8; i++) {
    forces[i] = pads.getForce(i);
  }

  /*
  telemetry.sendFloatArray("swerve/pad_raw_forces", rawForces, 8);
  telemetry.sendFloatArray("swerve/pad_forces", forces, 8);
  */

  telemetry.sendDouble("swerve/cop_x_l", pads.get_cop_x_l() / 255.0);
  telemetry.sendDouble("swerve/cop_y_l", pads.get_cop_y_l() / 255.0);
  telemetry.sendDouble("swerve/cop_x_r", pads.get_cop_x_r() / 255.0);
  telemetry.sendDouble("swerve/cop_y_r", pads.get_cop_y_r() / 255.0);
  telemetry.sendDouble("swerve/cop_x", pads.getX());
  telemetry.sendDouble("swerve/cop_y", pads.getY());
  // telemetry.sendDouble("swerve/cop_z", pads.getZ());
  // telemetry.sendDouble("swerve/total_weight", pads.getTotalWeight());
  // telemetry.sendFloatArray("swerve/steer_angles_deg", steerAngles, NUM_MODULES);
  // telemetry.sendFloatArray("swerve/drive_velocities", driveVelocities, NUM_MODULES);
  // telemetry.sendInt("swerve/mode", mode);
  // telemetry.sendBool("swerve/rc_connected", !sbusReceiver.rcLost());
  #pragma endregion
  
  #pragma region // Debug output every XXX ms
  static unsigned long lastDebugTime = 0;
  unsigned long currentTime = micros();
  if (currentTime - lastDebugTime >= 30000) {
    lastDebugTime = currentTime;
    

    float steerAngles[NUM_MODULES];
    float driveVelocities[NUM_MODULES];
    for (int i = 0; i < NUM_MODULES; i++) {
      steerAngles[i] = planner.getSteerAngle(i) * 180.0 / PI;  // Convert to degrees
      driveVelocities[i] = planner.getDriveWheelSpeed(i);
    }
    // print_load_cell_values();
    // print_pads_outputs();
    // print_pad_preload();
    // Serial.println();
  }
  #pragma endregion
}

void stopAllMotors() {
  // Stop local drive motors
  for (size_t i = 0; i < NUM_MODULES; i++) {
    if (allModules[i].remoteSlot >= 0) continue;
    allModules[i].driveMotor->setVelocity(0.0);
  }
  // Tell the slave to stop its motors (ENABLE flag clear)
  slavePacket.flags = 0;
  slavePacket.checksum = teensyLinkChecksum(slavePacket);
  Serial7.write((const uint8_t*)&slavePacket, sizeof(slavePacket));
}

void print_load_cell_values() {
  for (int i = 0; i < 8; i++) {
    Serial.print("Pad ");
    Serial.print(i);
    Serial.print(":");
    Serial.print(pads.getForce(i), 2);
    Serial.print(", ");
  }
  Serial.println();
}

void print_pad_preload(){

  double pl_left = pads.getRawForce(0) + pads.getRawForce(1) + pads.getRawForce(2) + pads.getRawForce(3);
  double pl_right = pads.getRawForce(4) + pads.getRawForce(5) + pads.getRawForce(6) + pads.getRawForce(7);
  char buf_left[20];
  snprintf(buf_left, sizeof(buf_left), "%8.1f", pl_left);
  char buf_right[20];
  snprintf(buf_right, sizeof(buf_right), "%8.1f", pl_right);

  Serial.println("Right preload: " + String(buf_right) + " N");
  Serial.println("Left preload: " + String(buf_left) + " N");
}

void print_pads_outputs() { 
  // Serial.print("X:");
  // Serial.print(pads.getX(), 2);
  // Serial.print(", Y:");
  // Serial.print(pads.getY(), 2);
  // Serial.print(", Z:");
  // Serial.print(pads.getZ(), 2);
  // Serial.println();
  double f_left = pads.getForce(0) + pads.getForce(1) + pads.getForce(2) + pads.getForce(3);
  double f_right = pads.getForce(4) + pads.getForce(5) + pads.getForce(6) + pads.getForce(7);
  Serial.print("Left:");
  Serial.print(f_left, 2);
  Serial.print(",");
  Serial.print("Right:");
  Serial.println(f_right, 2);
  Serial.print("Total weight:");
  Serial.println(f_left + f_right, 2);
  delay(50);
}
