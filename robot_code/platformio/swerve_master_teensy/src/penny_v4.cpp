#include <Arduino.h>
#include <math.h>
#include <FlexCAN_T4.h>
#include "Constants.h"
#include "ODrive.h"
#include "Vesc.h"
#include "SbusReceiver.h"
#include "shared/utils.h"
#include "SlewRateLimiter.h"
#include "ClosestAngleSteering.h"
#include "Planner.h"
#include "Swerve.h"
#include "SwerveTelemetry.h"
#include "Telemetry.h"
#include "Pads.h"

#pragma #region
// Function declarations
void controlSwerveModules();
void stopAllMotors();
void print_load_cell_values();
void print_pads_outputs();
void print_pad_preload();

// CAN Bus setup - Front modules on CAN3, Back modules on CAN1
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> frontCanBus;   // CAN3 for front modules (both steering and drive)
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> backCanBus;      // CAN1 for back modules (both steering and drive)

// Swerve Kinematics structs 
SwerveTrajectory traj;
pad_vars padVars;
SwerveKinematics kin;

// Simple swerve module structure using base class pointers
struct SwerveModule {
  ODriveBase* steerMotor;
  VescBase* driveMotor;
  String name;
  double steeringOffset;

  SwerveModule(ODriveBase* steer, VescBase* drive, const String& moduleName, double offset)
    : steerMotor(steer), driveMotor(drive), name(moduleName), steeringOffset(offset) {}
};

// Create individual motor instances
ODrive<CAN3> frontRightSteer(frontCanBus, 0);   // Front Right steering
Vesc<CAN3> frontRightDrive(frontCanBus, 10);    // Front Right drive
ODrive<CAN1> backRightSteer(backCanBus, 1);     // Back Right steering
Vesc<CAN1> backRightDrive(backCanBus, 11);      // Back Right drive
ODrive<CAN1> backLeftSteer(backCanBus, 2);      // Back Left steering
Vesc<CAN1> backLeftDrive(backCanBus, 12);       // Back Left drive
ODrive<CAN3> frontLeftSteer(frontCanBus, 3);    // Front Left steering
Vesc<CAN3> frontLeftDrive(frontCanBus, 13);     // Front Left drive

// Create array of all swerve modules
const size_t NUM_MODULES = 4;

SwerveModule allModules[NUM_MODULES] = {
  SwerveModule(&frontRightSteer, &frontRightDrive, "Front Right",  345 * M_PI / 180.0),  // [0]
  SwerveModule(&backRightSteer, &backRightDrive, "Back Right",     97 * M_PI / 180.0),  // [1]
  SwerveModule(&backLeftSteer, &backLeftDrive, "Back Left",       117 * M_PI / 180.0),  // [2]
  SwerveModule(&frontLeftSteer, &frontLeftDrive, "Front Left",    267 * M_PI / 180.0),  // [3]
};

// Timing
unsigned long lastControlUpdate = 0;
const unsigned long CONTROL_PERIOD = 2000;

SbusReceiver sbusReceiver;
SlewRateLimiter steerLimiter(PI * 4);
SlewRateLimiter driveLimiter(20);
ClosestAngleSteering closestAngleSteering;
Planner planner((double)CONTROL_PERIOD, traj, padVars, kin);
Telemetry telemetry;
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
  Serial.println("Penny V4 - Swerve Drive with ODrive Steering and VESC Drive");
  Serial.println("[INFO] Type BOOTLOAD via serial to trigger bootloader mode (no reset needed)");

  // Initialize CAN buses
  frontCanBus.begin();
  frontCanBus.setBaudRate(1000000);

  backCanBus.begin();
  backCanBus.setBaudRate(1000000);
  delay(100);

  // Initialize SBUS receiver
  sbusReceiver.init();

  // Initialize telemetry
  telemetry.start();

  // Initialize pads
  analogReadResolution(12); // Set ADC resolution to 12 bits (0-4095)
  pads.calibrate();
  Serial.println("Right preload: " + String(pads.getRawForce(0) + pads.getRawForce(1) + pads.getRawForce(2) + pads.getRawForce(3), 2) + " N");
  Serial.println("Left preload: " + String(pads.getRawForce(4) + pads.getRawForce(5) + pads.getRawForce(6) + pads.getRawForce(7), 2) + " N");

  // Configure ODrive motors for position control
  for (int i = 0; i < NUM_MODULES; i++) {
    allModules[i].steerMotor->setPositionControlMode();
    allModules[i].steerMotor->enableWithClosedLoop();
    delay(100);
  }

  Serial.println("Setup complete. Ready for control.");
  Serial.println("Left stick controls swerve modules:");
  Serial.println("- Stick direction sets steering angle");
  Serial.println("- Stick magnitude sets drive velocity");

  // swerveTelemetry.start();
}

void loop() {
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
  telemetry.retryIfNeeded();

  // Heartbeat: confirm the loop is running and report telemetry/network status
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat >= 1000 && false) {
    lastHeartbeat = millis();
    Serial.print("[HB] uptime_ms=");
    Serial.print(millis());
    Serial.print(" telemetryReady=");
    Serial.print(telemetry.isReady() ? "1" : "0");
    Serial.print(" link=");
    Serial.print(telemetry.linkStatus());  // 0=Unknown,1=On,2=Off
    Serial.print(" ip=");
    Serial.print(telemetry.localIp());
    Serial.print(" -> ");
    Serial.print(telemetry.remoteIp());
    Serial.print(":");
    Serial.print(telemetry.udpPort());
    Serial.print(" rcLost=");
    Serial.println(sbusReceiver.rcLost() ? "1" : "0");
  }

  unsigned long currentTime = micros();

  // Control loop at XXX Hz
  if (currentTime - lastControlUpdate >= CONTROL_PERIOD) {
    lastControlUpdate = currentTime;

    // Check if RC is connected
    if (sbusReceiver.rcLost()) {
      // Stop all motors if RC is lost
      stopAllMotors();
      Serial.println("RC Lost - Motors stopped");

      // Still send telemetry so network diagnostics and status updates
      // can be received even when the RC link is down.
      // telemetry.sendToSerial();
      return;
    }

    // Control all swerve modules
    controlSwerveModules();

    // Send telemetry data
    // telemetry.sendToSerial();
  }
}

double scaleWithDeadband(double input, double deadband) {
  if (abs(input) < deadband) {
    return 0.0;
  }
  return (input - copysign(deadband, input)) / (1.0 - deadband);
}

void controlSwerveModules() {
  // Read stick values
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
  if (blueSwitch > 0.5) { // tele-op
    mode = 0; 
  } else if (blueSwitch < 0.5 && blueSwitch > -0.5) {  // pads control
    mode = 1;
  } else {  // test mode 
    mode = 2;
  }

  // Apply commands to all modules
  for (int i = 0; i < NUM_MODULES; i++) {
    if (mode == 0) {
      planner.plan_teleop(rightStickX, rightStickY, leftStickX, gain_in);
    } else if (mode == 1) {
      planner.plan_pads(pads.getX(), pads.getY(), pads.getZ(), gain_in);
    } else {
      Serial.println("Zeroing mode ");
      pads.tare();
    }

    if (!disableTurning) {
      allModules[i].steerMotor->setPosition(planner.getSteerAngle(i)/(2.0 * PI) + allModules[i].steeringOffset/(2.0 * PI));
    }
    allModules[i].driveMotor->setVelocity(disableDriving ? 0 : planner.getDriveWheelSpeed(i));
    delayMicroseconds(100);
    // Serial.println(allModules[i].name + " - Steer Pos: " + String(planner.getSteerAngle(i) * 180.0 / PI, 2) + " deg, Drive Vel: " + String(planner.getDriveWheelSpeed(i), 2) + " m/s");
  }

  // Debug output every XXX ms
  static unsigned long lastDebugTime = 0;
  unsigned long currentTime = micros();
  if (currentTime - lastDebugTime >= 30000) {
    lastDebugTime = currentTime;
    pads.calcVector();
    
    // Send pad telemetry data
    // Raw load cell forces (8 sensors)
    float rawForces[8];
    for (int i = 0; i < 8; i++) {
      rawForces[i] = pads.getRawForce(i);
    }
    telemetry.sendFloatArray("swerve/pad_raw_forces", rawForces, 8);
    
    // Actual load cell forces (8 sensors, zeroed)
    float forces[8];
    for (int i = 0; i < 8; i++) {
      forces[i] = pads.getForce(i);
    }
    telemetry.sendFloatArray("swerve/pad_forces", forces, 8);
    
    // Center of pressure (X, Y, Z normalized outputs)
    telemetry.sendDouble("swerve/cop_x", pads.getX());
    telemetry.sendDouble("swerve/cop_y", pads.getY());
    telemetry.sendDouble("swerve/cop_z", pads.getZ());
    
    // Total weight
    telemetry.sendDouble("swerve/total_weight", pads.getTotalWeight());
    
    // Send swerve module telemetry data
    float steerAngles[NUM_MODULES];
    float driveVelocities[NUM_MODULES];
    for (int i = 0; i < NUM_MODULES; i++) {
      steerAngles[i] = planner.getSteerAngle(i) * 180.0 / PI;  // Convert to degrees
      driveVelocities[i] = planner.getDriveWheelSpeed(i);
    }
    telemetry.sendFloatArray("swerve/steer_angles_deg", steerAngles, NUM_MODULES);
    telemetry.sendFloatArray("swerve/drive_velocities", driveVelocities, NUM_MODULES);
    
    // System status
    telemetry.sendInt("swerve/mode", mode);
    telemetry.sendBool("swerve/rc_connected", !sbusReceiver.rcLost());
    telemetry.sendBool("swerve/motors_enabled", !disableDriving && !disableTurning);
    
    // print_load_cell_values();
    print_pads_outputs();
    // print_pad_preload();
    // Serial.println();
  }
}

void stopAllMotors() {
  // Stop all drive motors
  for (int i = 0; i < NUM_MODULES; i++) {
    allModules[i].driveMotor->setVelocity(0.0);
  }
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
