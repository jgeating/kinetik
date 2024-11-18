#include <math.h>             // Math functions
#include "Kinematics.h"       // wheel level kinematics/trigonometry
#include "Planner.h"          // robot level planning
#include "shared/utils.h"     // Basic utils like more powerful serial
#include "penny/Steer.h"      // For controlling steering actuator
#include "penny/Pads.h"       // For interfacing with weight pads
#include "penny/Drive.h"      // For controlling drive motors
#include <Wire.h>             // For accessing native Arduino I2C functions
#include "Adafruit_Sensor.h"  // Downloaded library for IMU stuff
#include "Adafruit_BNO055.h"  // Downloaded library for IMU stuff
#include "utility/imumaths.h" // Downloaded library for IMU stuff
#include "Swerve.h"
#include "Performance.h"
#include "PID.h"                    // For PID controllers
#include "shared/LowPassFilter.cpp" // Low pass filter class
#include "penny/Lights.h"
#include "SwerveTelemetry.h"
#include "SbusReceiver.h"

// Definitions
#pragma region

#define RADIUS_SWERVE_ASSEMBLY 0.25 // distance to wheel swerve axes, meters
#define DEAD_ZONE 0.1
#define pi 3.14159265358979
#define TELEMETRY_REPORT_PERIOD 500000
#define MCU "DUE" // Either "T4_1", or "DUE"

// Instantiate structs
SwerveTrajectory traj;
pad_vars padVars;
SwerveKinematics kin;
LoopTiming loopTiming;
Profiles profiles;
Modes modes; // Handles different driving and control modes

// Instantiate classes
Planner *planner; // for planning robot level motion
Pads *pads;       // For driving with force pads
PID *padx_pid;    // PID controller for weight (pad) steering in X (sideways) axis
PID *pady_pid;    // PID controller for weight (pad) steering in Y (forwards) axis
PID *padz_pid;    // PID controller for weight (pad) steering in Z (rotation) axis
Lights *lights;   // Controls LED strips for signals/entertainment

SwerveCAN can;
Drive::Type types[] = {Drive::Type::ODRIVE, Drive::Type::ODRIVE, Drive::Type::ODRIVE, Drive::Type::ODRIVE};

RobotState robotState;
Drive **drive = new Drive *[kin.nWheels];
Steer **steer = new Steer *[kin.nWheels];
double aMaxSteer = 5000;  // Max angular acceleration of steer motor, in motor frame, rad/s^2. Safe starting value: 5000
double wMaxSteer = 10000; // Max angular velocity of steer motor, in motor frame, rad/s. Safe starting value: 10000
int doneHoming = 0;       // Used to determine when calibration sequence is finished. 1 = finished.

// Plotting, telemetry
int plotCounter = 0;
unsigned long prevTelemetryReportTime = 0;
Watchdog watchdog;
LowPassFilter filter(10); // create a low-pass filter with 10 readings

#pragma endregion

SwerveTelemetry swerveTelemetry;
SbusReceiver sbusReceiver;

void setup()
{
  // Serial and CAN setup
  Serial.begin(460800); // Bumping up serial rate 7/21/2024 for serial telemetry over usb to computer
  delay(400);

  analogReadResolution(12);

  motors::canBus1.begin();
  motors::canBus1.setBaudRate(1000000);

  delay(500);

  for (int i = 0; i < 4; i++) {
    motors::steer[i].printMessage();
  }

  sbusReceiver.init();

  for (int i = 0; i < 4; i++)
  {
    // This is causing CAN stuff on the steering motors not to work for some reason
    // pinMode(robotState.irPin[i], INPUT);
  }

  // Set up digital I/O
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW); // Set up indicator LED

  Serial.println("CAN and Pins initialized. Setting up kinematics and trajectory planning objects...");

  delay(3000); // Need a delay between CAN initialization and ODRIVE initialization (in the Drive class)

  // Kinematics and path planning setup
  kin.dRatio = kin.pole_pairs * 60 / (2 * M_PI) / (.083 / 2); // used to convert m/s to rpm
  for (int i = 0; i < kin.nWheels; i++)
  {
    robotState.irPos[i] = robotState.irPos[i]; // Correcting for polar coordinate frame
    steer[i] = new Steer(wMaxSteer, aMaxSteer, kin.yRatio, loopTiming.tInner, can.len, i);
    drive[i] = new Drive(traj.qd_max[0], traj.qdd_max[0], kin.dRatio, loopTiming.tInner, can.len, i, types[i]);
    kin.kinematics[i] = new Kinematics(RADIUS_SWERVE_ASSEMBLY, DEAD_ZONE, i);
  }

  // Set up pad pid classes
  double dt = loopTiming.tInner / 1000000.0;
  pads = new Pads();
  padx_pid = new PID(padVars.kp[0], padVars.ki[0], padVars.kd[0], dt, padVars.lag[0]);
  pady_pid = new PID(padVars.kp[1], padVars.ki[1], padVars.kd[1], dt, padVars.lag[1]);
  padz_pid = new PID(padVars.kp[2], padVars.ki[2], padVars.kd[2], dt, padVars.lag[2]);
  // lights = new Lights(2, 65);

  padx_pid->setSetpoint(0); // setpoint = 0 means try to put human center of pressure at middle of footpad
  pady_pid->setSetpoint(0);
  padz_pid->setSetpoint(0);

  planner = new Planner(loopTiming.tInner, traj, padVars, kin);

  pads->calibrate(); // Zero footpads, assuming zero weight on them

  // swerveTelemetry.start();

  Serial.println("Startup Complete.");
  delay(1000);
}

void teleop()
{
  for (int k = 0; k < 4; k++)
  {
    if (k == 0) {
      traj.input[k] = constrain(sbusReceiver.getRightHor(), -1.0, 1.0);
    } else if (k == 1) {
      traj.input[k] = constrain(sbusReceiver.getRightVert(), -1.0, 1.0);
    } else if (k == 2) {
      traj.input[k] = constrain(sbusReceiver.getLeftHor(), -1.0, 1.0);
    } else {
      traj.input[k] = constrain(sbusReceiver.getLeftVert() * .5, -0.5, 0.5) + 0.5;
    }
  }
  planner->plan_teleop(traj.input[0], traj.input[1], traj.input[2], traj.input[3]);
}

void padRiding()
{
  double hand_remote_val = constrain(sbusReceiver.getHandheld() * .5, -0.5, 0.5) + 0.5; // ch 3 rewired to read value from handheld e-skate remote
  bool hand_remote_estopped = hand_remote_val < 0.2 && hand_remote_val > -0.2;
  bool hand_zeroing = hand_remote_val <= -0.2;
  bool hand_active_driving = hand_remote_val > 0.2;
  pads->calcVector();

  double x = pads->getX();
  double y = pads->getY();
  double z = pads->getZ();

  planner->plan_pads(x, y, z, hand_remote_val);
  if (sbusReceiver.isBlueSwitchDown() || modes.zeroing || hand_zeroing || hand_remote_estopped)
  {
    planner->eStop();
  }
}

void updateLoopTiming()
{
  if (loopTiming.behind)
  { // This means we can't keep up with the desired loop rate. Trip LED to indicate so
    digitalWrite(13, HIGH);
    loopTiming.lastInner = loopTiming.now;
  }
  else
  {
    digitalWrite(13, LOW);
    loopTiming.lastInner = loopTiming.lastInner + loopTiming.tInner;
    loopTiming.behind = true;
  }
}

void zeroFootPads()
{
  if (!modes.zeroing)
  {
    Serial.println("Zeroing entered");
    modes.zeroing = true;
  }
  switch (modes.mode)
  {
  case Mode::WEIGHT_CONTROL:
    Serial.println("IMU zeroing not supported");
    break;
  case Mode::PADS:
    // Serial.println("Pads zeroing");
    pads->calibrate();
    planner->eStop();
    break;
  default:
    Serial.println("Unable to parse mode to enter zeroing. This is probably a bug");
  }
}

void loop()
{
  // startProfile(profiles.robotLoop);
  // printWatchdogError(watchdog);
  sbusReceiver.read();
  telemetry();

  loopTiming.now = micros();

  if (loopTiming.now - loopTiming.lastInner > loopTiming.tInner)
  {
    updateLoopTiming();

    switch (modes.mode)
    { // Primary case statement for handling modes
    case Mode::TELEOP:
      teleop();
      break;
    case Mode::PADS:
      padRiding();
      break;
    }
    for (int i = 0; i < kin.nWheels; i++)
    {        
      steer[i]->motTo(planner->getMotAngle(i), planner->getMotSteerVel(i), sbusReceiver.getRedSwitch(), sbusReceiver.rcLost()); // Red, (-) is up
      delayMicroseconds(can.steerCanDelay);                                                     // Nasty bug where going from 3 motors to 4 per bus required a 100 us delay instead of 50
      drive[i]->setVel(-planner->getDriveWheelSpeed(i), sbusReceiver.getRedSwitch(), sbusReceiver.rcLost());
      delayMicroseconds(can.driveCanDelay);
    }
  }
  else // Overtiming
  {
    loopTiming.behind = false;
  }

  if (loopTiming.now - loopTiming.lastOuter > loopTiming.tOuter) // Slow loop
  {
    loopTiming.lastOuter = loopTiming.lastOuter + loopTiming.tOuter;
    //***********************BEGIN SLOW LOOP*******************************

    // Check channels, modes
    if (sbusReceiver.getRightKnob() > 400)
    {
      calMotor(can); // Calibrate motors
    }

    if (sbusReceiver.isBlueSwitchUp())
    { // default mode is tele-op, blue stick top position
      if (modes.mode != Mode::TELEOP)
      {
        Serial.println("Mode 0 entered (tele-op)");
      }
      modes.mode = Mode::TELEOP;
      planner->setMode(0);
      modes.eStop = false; // Also disable e-stop if tripped
      modes.zeroing = false;
    }

    if (sbusReceiver.isBlueSwitchCentered()) // Mode switch is centered
    {                                       // IMU zeroing mode
      if (modes.mode != Mode::PADS)
      {
        Serial.println("Mode 2 entered (pad steering)");
        modes.mode = Mode::PADS;
        delay(100);
      }
      modes.zeroing = false;
    }

    // remote pulled back or transmitter in zeroing mode
    if (sbusReceiver.getHandheld() < -.1 || sbusReceiver.isBlueSwitchDown())
    {
      zeroFootPads();
    }

    if (sbusReceiver.getRedSwitch() < .9 || sbusReceiver.rcLost())
    { // Safety loop. This runs if motors aren't meant to be spinning
      // Serial.println("Shutting off ODrive Motor ID 1");
      for (int i = 0; i < 4; i++)
      {
        motors::drive[i].setVelocity(0);
      }
    }
  }
  // printProfiles(profiles);
}

// Helper Functions
#pragma region
// CALIBRATION
void calMotor(SwerveCAN &can)
{
  for (int j = 0; j < kin.nWheels; j++)
  {
    steer[j]->setHoming(2); // set homing mode to true for all axes
    steer[j]->motTo(0, sbusReceiver.getRedSwitch(), sbusReceiver.rcLost());
  }
  delay(1000);           // Give motor time to move to zero position if it is wound up
  double fineTune = 1.0; // to step in less than 1 deg increments - this is the ratio (0.2 would be in 0.2 degree increments
  for (int i = 0; i < (int)(360 * abs(kin.yRatio) * 1.5 / fineTune); i++)
  {
    for (int j = 0; j < kin.nWheels; j++)
    {
      can.pos = i * fineTune;
      if (steer[j]->getHoming() == 2 && digitalRead(robotState.irPin[j]) == 1)
      { // calibration started with IR triggered
        steer[j]->setHoming(1);
      }
      if (steer[j]->getHoming() == 1 && digitalRead(robotState.irPin[j]) == 0)
      { // Hit target fresh
        planner->setSteerAngle(robotState.irPos[j] * PI / 180.0, j);
        planner->setMotAngle(can.pos * PI / 180.0, j);
        // steer[j]->setYaw(robotState.irPos[j]);
        // steer[j]->setMPos(can.pos);
        steer[j]->setHoming(0);
      }
      if (steer[j]->getHoming() == 0)
      { // position signal should persist, but motor should stop moving after cal marker is detected
        can.pos = steer[j]->getMPos();
      }
      steer[j]->motTo(can.pos * PI / 180.0, sbusReceiver.getRedSwitch(), sbusReceiver.rcLost());
    }
    delayMicroseconds(800);
    doneHoming = 1;
    for (int j = 0; j < kin.nWheels; j++)
    {
      doneHoming = doneHoming && steer[j]->getHoming() == 0;
    }
    if (doneHoming == 1)
    {
      kin.calibrated = 1; // set calibration flag to true
    }
    if (doneHoming == 1)
      // while (1){}  % used to freeze homing positions after cal to measure offsets with phone/level
      break; // break from loop once all targets have been found
  }
}
double mapDouble(double x, double min_in, double max_in, double min_out, double max_out)
{
  double ret = (x - min_in) / (max_in - min_in);
  ret = ret * (max_out - min_out) + min_out;
  return ret;
}

void telemetry()
{
  unsigned long time = micros();
  if (time - prevTelemetryReportTime < TELEMETRY_REPORT_PERIOD)
  {
    return;
  }
  prevTelemetryReportTime = time;

  // Serial.println("\n");
}

#pragma endregion
