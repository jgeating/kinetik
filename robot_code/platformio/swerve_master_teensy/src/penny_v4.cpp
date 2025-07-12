#include <Arduino.h>
#include <math.h>
#include <FlexCAN_T4.h>
#include "Constants.h"
#include "ODrive.h"
#include "Vesc.h"
#include "SbusReceiver.h"
#include "shared/utils.h"

// Function declarations
void controlSwerveModules();
void stopAllMotors();

// CAN Bus setup - Front modules on CAN3, Back modules on CAN1
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> frontCanBus;   // CAN3 for front modules (both steering and drive)
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> backCanBus;      // CAN1 for back modules (both steering and drive)

// Simple swerve module structure using base class pointers
struct SwerveModule {
  ODriveBase* steerMotor;
  VescBase* driveMotor;
  String name;

  SwerveModule(ODriveBase* steer, VescBase* drive, const String& moduleName)
    : steerMotor(steer), driveMotor(drive), name(moduleName) {}
};

// Create individual motor instances
ODrive<CAN2> frontRightSteer(frontCanBus, 0);   // Front Right steering
Vesc<CAN2> frontRightDrive(frontCanBus, 10);    // Front Right drive
ODrive<CAN1> backRightSteer(backCanBus, 1);       // Back Right steering
Vesc<CAN1> backRightDrive(backCanBus, 11);        // Back Right drive
ODrive<CAN1> backLeftSteer(backCanBus, 2);        // Back Left steering
Vesc<CAN1> backLeftDrive(backCanBus, 12);         // Back Left drive
ODrive<CAN2> frontLeftSteer(frontCanBus, 3);    // Front Left steering
Vesc<CAN2> frontLeftDrive(frontCanBus, 13);     // Front Left drive

// Create array of all 4 swerve modules
const size_t NUM_MODULES = 4;

SwerveModule allModules[NUM_MODULES] = {
  SwerveModule(&frontRightSteer, &frontRightDrive, "Front Right"),  // [0]
  SwerveModule(&backRightSteer, &backRightDrive, "Back Right"),     // [1]
  SwerveModule(&backLeftSteer, &backLeftDrive, "Back Left"),        // [2]
  SwerveModule(&frontLeftSteer, &frontLeftDrive, "Front Left"),      // [3]
};

// SBUS Receiver
SbusReceiver sbusReceiver;

// Timing
unsigned long lastControlUpdate = 0;
const unsigned long CONTROL_PERIOD = 20000; // 20ms = 50Hz

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Penny V4 - Swerve Drive with ODrive Steering and VESC Drive");

  // Initialize CAN buses
  frontCanBus.begin();
  frontCanBus.setBaudRate(1000000);

  backCanBus.begin();
  backCanBus.setBaudRate(1000000);
  delay(1000);

  // Initialize SBUS receiver
  sbusReceiver.init();

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
}

void loop() {
  // Read SBUS data
  sbusReceiver.read();

  unsigned long currentTime = micros();

  // Control loop at 50Hz
  if (currentTime - lastControlUpdate >= CONTROL_PERIOD) {
    lastControlUpdate = currentTime;

    // Check if RC is connected
    if (sbusReceiver.rcLost()) {
      // Stop all motors if RC is lost
      stopAllMotors();
      Serial.println("RC Lost - Motors stopped");
      return;
    }

    // Control all swerve modules
    controlSwerveModules();
  }
}

double stickAngle = 0.0;
const double STEERING_DEADZONE = 0.1;
const double MAX_VELOCITY = 2.0; // m/s - adjust based on your robot's capabilities

void controlSwerveModules() {
  // Read left stick values
  double leftStickX = sbusReceiver.getLeftHor();  // -1.0 to 1.0
  double leftStickY = sbusReceiver.getLeftVert(); // -1.0 to 1.0
  double rightStickX = sbusReceiver.getRightHor();  // -1.0 to 1.0
  double rightStickY = sbusReceiver.getRightVert(); // -1.0 to 1.0

  // Calculate stick magnitude and angle
  double speed = constrain((leftStickY + 1.0) / 2.0, 0.0, 1.0);
  
  // Calculate angle in radians (0 = forward, positive = clockwise)
  double nextStickAngle = atan2(rightStickX, rightStickY);
  double stickAngleMagnitude = sqrt(rightStickX * rightStickX + rightStickY * rightStickY);

  if (stickAngleMagnitude > STEERING_DEADZONE) {
    stickAngle = nextStickAngle;
  }

  // Convert stick angle to position command for steering motors
  // ODrive position is in revolutions, so convert radians to revolutions
  double steerPosition = stickAngle / (2.0 * PI);

  // Convert stick magnitude to velocity for drive motors
  // Scale magnitude to reasonable velocity (adjust as needed)
  double driveVelocity = speed * MAX_VELOCITY;

  double redSwitch = sbusReceiver.getRedSwitch();
  bool disableTurning = redSwitch < -0.5;
  bool disableDriving = redSwitch < 0.5;
  
  // Apply commands to all modules
  for (int i = 0; i < NUM_MODULES; i++) {
    if (!disableTurning) {
      allModules[i].steerMotor->setPosition(steerPosition);
    }
    allModules[i].driveMotor->setVelocity(disableDriving ? 0 : driveVelocity);
    delayMicroseconds(400);
  }

  // Debug output every 1 second
  static unsigned long lastDebugTime = 0;
  unsigned long currentTime = micros();
  if (currentTime - lastDebugTime >= 1000000) {
    lastDebugTime = currentTime;
    Serial.print("Right Stick X: "); 
    Serial.print(rightStickX, 3);
    Serial.print("Right Stick Y: "); 
    Serial.print(rightStickY, 3);
    Serial.print(", Left Stick Y: "); 
    Serial.print(leftStickY, 3);
    Serial.print(", Angle: ");
    Serial.println(stickAngle * 180.0 / PI, 1);
    Serial.print(", Velocity: "); 
    Serial.print(driveVelocity, 3);
    Serial.print(", Red Switch: ");
    Serial.println(redSwitch);
  }
}

void stopAllMotors() {
  // Stop all drive motors
  for (int i = 0; i < NUM_MODULES; i++) {
    allModules[i].driveMotor->setVelocity(0.0);
  }
}