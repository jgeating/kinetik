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

// Function declarations
void controlSwerveModules();
void stopAllMotors();

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
  SwerveModule(&frontRightSteer, &frontRightDrive, "Front Right", 0 * M_PI / 180.0),  // [0]
  SwerveModule(&backRightSteer, &backRightDrive, "Back Right", 0 * M_PI / 180.0),     // [1]
  SwerveModule(&backLeftSteer, &backLeftDrive, "Back Left", 0 * M_PI / 180.0),        // [2]
  SwerveModule(&frontLeftSteer, &frontLeftDrive, "Front Left", 0 * M_PI / 180.0),      // [3]
  // SwerveModule(&frontRightSteer, &frontRightDrive, "Front Right", 68 * M_PI / 180.0),  // [0]
  // SwerveModule(&backRightSteer, &backRightDrive, "Back Right", -5 * M_PI / 180.0),     // [1]
  // SwerveModule(&backLeftSteer, &backLeftDrive, "Back Left", 92 * M_PI / 180.0),        // [2]
  // SwerveModule(&frontLeftSteer, &frontLeftDrive, "Front Left", 70 * M_PI / 180.0),      // [3]
};

// Timing
unsigned long lastControlUpdate = 0;
const unsigned long CONTROL_PERIOD = 2000;

SbusReceiver sbusReceiver;
SlewRateLimiter steerLimiter(PI * 4);
SlewRateLimiter driveLimiter(20);
ClosestAngleSteering closestAngleSteering;
Planner planner((double)CONTROL_PERIOD, traj, padVars, kin);
SwerveTelemetry swerveTelemetry;

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("Penny V4 - Swerve Drive with ODrive Steering and VESC Drive");

  // Initialize CAN buses
  frontCanBus.begin();
  frontCanBus.setBaudRate(1000000);

  backCanBus.begin();
  backCanBus.setBaudRate(1000000);
  delay(100);

  // Initialize SBUS receiver
  sbusReceiver.init();

  // Initialize telemetry
  swerveTelemetry.start();

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

  // Control loop at XXX Hz
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

    // Send buffered telemetry data
    swerveTelemetry.sendBufferedData();
  }
}

double stickAngle = 0.0;
const double STEERING_DEADZONE = 0.25;
const double MAX_VELOCITY = 10; // m/s - adjust based on your robot's capabilities

double scaleWithDeadband(double input, double deadband) {
  if (abs(input) < deadband) {
    return 0.0;
  }
  return (input - copysign(deadband, input)) / (1.0 - deadband);
}

void controlSwerveModules() {
  // Read left stick values
  double leftStickX = sbusReceiver.getLeftHor();  // -1.0 to 1.0
  double leftStickY = sbusReceiver.getLeftVert(); // -1.0 to 1.0
  double rightStickX = sbusReceiver.getRightHor();  // -1.0 to 1.0
  double rightStickY = sbusReceiver.getRightVert(); // -1.0 to 1.0

  // Calculate stick magnitude and angle
  double speed = constrain((leftStickY + 1.0) / 2.0, 0.0, 1.0);

  double vx = rightStickX;
  double vy = rightStickY;
  double vz = leftStickX;
  double gain_in = speed;

  // Calculate angle in radians (0 = forward, positive = clockwise)
  // double nextStickAngle = atan2(rightStickX, rightStickY);
  // double stickAngleMagnitude = sqrt(rightStickX * rightStickX + rightStickY * rightStickY);

  // if (stickAngleMagnitude > STEERING_DEADZONE) {
  //   stickAngle = nextStickAngle;
  // }

  // Convert stick magnitude to velocity for drive motors
  // Scale magnitude to reasonable velocity (adjust as needed)
  // double driveVelocity = speed * MAX_VELOCITY * scaleWithDeadband(stickAngleMagnitude, STEERING_DEADZONE);
  // double slewedDriveVelocity = driveLimiter.calculate(driveVelocity);

  double redSwitch = sbusReceiver.getRedSwitch();
  bool disableTurning = redSwitch < -0.5;
  bool disableDriving = redSwitch < 0.5;

  double closestStickAngle = closestAngleSteering.calculate(stickAngle);
  double closestSteerPosition = closestStickAngle / (2.0 * PI);
  
  // double slewedstickAngle = steerLimiter.calculate(stickAngle);
  // double closestStickAngle = closestAngleSteering.calculate(slewedstickAngle);

  // // Convert stick angle to position command for steering motors
  // // ODrive position is in revolutions, so convert radians to revolutions
  // double closestSteerPosition = closestStickAngle / (2.0 * PI);

  // Apply commands to all modules
  for (int i = 0; i < NUM_MODULES; i++) {
    // if (i == 0 || i == 3) { // Front modules use CAN3
      planner.plan_teleop(vx, vy, vz, gain_in);

      if (!disableTurning) {
        allModules[i].steerMotor->setPosition(planner.getSteerAngle(i)/(2.0 * PI) + allModules[i].steeringOffset/(2.0 * PI));
      }
      allModules[i].driveMotor->setVelocity(disableDriving ? 0 : planner.getDriveWheelSpeed(i));
      delayMicroseconds(100);
      Serial.println(allModules[i].name + " - Steer Pos: " + String(planner.getSteerAngle(i) * 180.0 / PI, 2) + " deg, Drive Vel: " + String(planner.getDriveWheelSpeed(i), 2) + " m/s");
    // }
  }

  // Debug output every 1 second
  static unsigned long lastDebugTime = 0;
  unsigned long currentTime = micros();
  if (currentTime - lastDebugTime >= 100000) {
    lastDebugTime = currentTime;
    // Serial.print("Right Stick X: "); 
    // Serial.print(rightStickX, 3);
    // Serial.print("Right Stick Y: "); 
    // Serial.print(rightStickY, 3);
    // Serial.print(", Left Stick Y: "); 
    // Serial.print(leftStickY, 3);
    // Serial.print(", Angle: ");
    // Serial.println(stickAngle * 180.0 / PI, 1);
    // Serial.print(", Velocity: "); 
    // Serial.print(driveVelocity, 3);
    // Serial.print(", Red Switch: ");
    // Serial.println(redSwitch);
    // swerveTelemetry.queueDouble(planner.getDriveWheelSpeed(3));
    // Serial.println(planner.getDriveWheelSpeed(3));  // Keep local debug
    // Serial.println("----");
  }
}

void stopAllMotors() {
  // Stop all drive motors
  for (int i = 0; i < NUM_MODULES; i++) {
    allModules[i].driveMotor->setVelocity(0.0);
  }
}
