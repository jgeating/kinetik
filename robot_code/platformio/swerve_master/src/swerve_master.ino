#include "DueCANLayer.h";     // CAN library for CAN shield
#include <math.h>;            // Math functions
#include "Kinematics.h";      // wheel level kinematics/trigonometry
#include "Planner.h";         // robot level planning
#include "shared/utils.h";    // Basic utils like more powerful serial
#include "penny/Steer.h";     // For controlling steering actuator
#include "penny/Pads.h";      // For interfacing with weight pads
#include "penny/Drive.h";     // For controlling drive motors
#include <Wire.h>;            // For accessing native Arduino I2C functions
#include "Adafruit_Sensor.h"  // Downloaded library for IMU stuff
#include "Adafruit_BNO055.h"  // Downloaded library for IMU stuff
#include "utility/imumaths.h" // Downloaded library for IMU stuff
#include "Swerve.h";
#include "Performance.h";
#include "PID.h";                    // For PID controllers
#include "shared/LowPassFilter.cpp"; // Low pass filter class

// Definitions
#pragma region

const int CHANNEL_PIN[] = {
    38, // left stick vertical, forward = (+)
    40, // right stick horizontal, R = (+)
    42, // right stick vertical, forward = (+)
    44, // left stick horizontal, R = (+)
    46, // left knob, CW = (+)
    48, // right knob, CW = (+)
    50, // left switch, backwards = (-)
    52  // right switch, backwards = (-)
};
int rctemp[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Temp storage for rc value debugging

#define RADIUS_SWERVE_ASSEMBLY 0.25 // distance to wheel swerve axes, meters
#define DEAD_ZONE 0.1
#define pi 3.14159265358979
#define TELEMETRY_REPORT_PERIOD 500000

// General stuff, controls
char buff[100] = ""; // For various sprintf print outs
double qd_d = 0;     // temporary used for testing vest acceleration control. Stores instantaneous velocity
double qdd_d = 0;
double temp = 0; // generic doubles for testing new things
double temp2 = 0;
double temp3 = 0;
double global_gain = 0; // Used for various things 
double dt = 0;        // loop period. Pulled from timing loop parameter, converted from usec to sec
int bringupMode = -1; // Bringup mode. set to -1 for normal operation, 0... are for bringing up specific axes for single DOF PID tuning 0 = X, 1 = Y, 2 = Z

// Instantiate structs
SwerveTrajectory traj;
pad_vars padVars;
SwerveKinematics kin;
LoopTiming loopTiming;
Profiles profiles;
Modes modes; // Handles different driving and control modes

// Instantiate classes?
Planner *planner; // for planning robot level motion
Pads *pads;       // For driving with force pads
PID *padx_pid;
PID *pady_pid;
PID *padz_pid;

// CAN Stuff for drive motors
extern byte canInit(byte cPort, long lBaudRate);
extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte *cData, byte cDataLen);
extern byte canRx(byte cPort, long *lMsgID, bool *bExtendedFormat, byte *cData, byte *cDataLen);
SwerveCAN can;
Drive::Type types[] = {Drive::Type::VESC, Drive::Type::VESC, Drive::Type::ODRIVE, Drive::Type::VESC};

// PWM/Receiver stuff
PWMReceiver pwmReceiver;
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

void setup()
{
  analogReadResolution(12);
  // Serial and CAN setup
  Serial.begin(115200);
  Serial.println("Initializing CAN and pinmodes...");
  if (canInit(0, CAN_BPS_1000K) == CAN_OK)
    Serial.print("CAN0: Initialized Successfully.\n\r");
  else
    Serial.print("CAN0: Initialization Failed.\n\r");
  if (canInit(1, CAN_BPS_1000K) == CAN_OK)
    Serial.print("CAN1: Initialized Successfully.\n\r");
  else
    Serial.print("CAN1: Initialization Failed.\n\r");
  for (int i = 0; i < 4; i++)
  {
    pinMode(robotState.irPin[i], INPUT);
  }

  // Set up digital I/O
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW); // Set up indicator LED
  Serial.println("CAN and Pins initialized. Initializing RC interrupts...");

  // RC PWM interrupt setup
  for (int i = 0; i < pwmReceiver.chs; i++)
  {
    pwmReceiver.channels[i] = new Channel(CHANNEL_PIN[i], pwmReceiver.chOff[i]);
  }
  attachInterrupt(pwmReceiver.channels[0]->getPin(), calcCh1, CHANGE);
  attachInterrupt(pwmReceiver.channels[1]->getPin(), calcCh2, CHANGE);
  attachInterrupt(pwmReceiver.channels[2]->getPin(), calcCh3, CHANGE);
  attachInterrupt(pwmReceiver.channels[3]->getPin(), calcCh4, CHANGE);
  attachInterrupt(pwmReceiver.channels[4]->getPin(), calcCh5, CHANGE);
  attachInterrupt(pwmReceiver.channels[5]->getPin(), calcCh6, CHANGE);
  attachInterrupt(pwmReceiver.channels[6]->getPin(), calcCh7, CHANGE);
  attachInterrupt(pwmReceiver.channels[7]->getPin(), calcCh8, CHANGE);
  delay(100); // Wait for rc channels to initially detect, before zeroing
  // for (int i = 1; i < 4; i++){  // Zero spring centered joysticks 
  //   pwmReceiver.channels[i]->zero();
  // }
  Serial.println("RC interrrupts initialized. Setting up kinematics and trajectory planning objects...");

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
  dt = loopTiming.tInner / 1000000.0;
  pads = new Pads();
  padx_pid = new PID(padVars.kp[0], padVars.ki[0], padVars.kd[0], dt, padVars.lag[0]);
  pady_pid = new PID(padVars.kp[1], padVars.ki[1], padVars.kd[1], dt, padVars.lag[1]);
  padz_pid = new PID(padVars.kp[2], padVars.ki[2], padVars.kd[2], dt, padVars.lag[2]);

  padx_pid->setSetpoint(0); // setpoint = 0 means try to put human center of pressure at middle of footpad
  pady_pid->setSetpoint(0);
  padz_pid->setSetpoint(0);

  planner = new Planner(loopTiming.tInner, traj, padVars, kin);

  Serial.println("Startup Complete.");
}

void loop()
{
  // startProfile(profiles.robotLoop);
  // printWatchdogError(watchdog);
  // telemetry();
  loopTiming.now = micros();
  if (loopTiming.now - loopTiming.lastInner > loopTiming.tInner)
  {
    { // loop timing 
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

    switch (modes.mode){  // Primary case statement for handling modes
      case 0: // ********************************* Tele-op mode *********************************
        for (int k = 0; k < 4; k++){
          if (k != 3){
            traj.input[k] = constrain(pwmReceiver.channels[k + 1]->getCh() / 500.0, -1.0, 1.0);
          } else {
            traj.input[3] = constrain(pwmReceiver.channels[0]->getCh() / 1000.0, -0.5, 0.5) + 0.5;
          }
        }
        planner->plan_teleop(traj.input[0], traj.input[1], traj.input[2], traj.input[3]);
        break;
      case 2: // ********************************* Pad riding mode *********************************
        double a_max = 15;     // limit max acceleration, m/s^2
        double v_max = 10;     // limit max velocity
        // double alpha_max = 20; // limit max angular acceleration, rad/s^2
        double w_max = 30;     // limit max angular velocity, rad/s

        pads->calcVector();
        padx_pid->setInput(pads->getX());
        pady_pid->setInput(pads->getY());
        padz_pid->setInput(pads->getZ());

        // qdd_d = traj.input[0] * padVars.kp_x;    // driving without PID controller, just P controller
        // qdd_d = constrain(qdd_d, -a_max, a_max);
        traj.qdd_d[0] = constrain(padx_pid->compute(), -a_max, a_max);
        traj.qd_d[0] = constrain(traj.qd_d[0] - traj.qdd_d[0] * dt, -v_max, v_max);
        // qd_d = constrain(traj.input[0] * padVars.kp_x, -v_max, v_max);   // velocity control. useful for bringup
        // qdd_d = traj.input[1] * padVars.kp_y;    // driving without PID controller, just P controller
        // qdd_d = constrain(qdd_d, -a_max, a_max);

        // traj.qdd_d[1] = constrain(pady_pid->compute(), -a_max, a_max);
        // traj.qd_d[1] = constrain(traj.qd_d[1] + traj.qdd_d[1] * dt, -v_max, v_max);
        traj.qd_d[1] = 0;
        // traj.qd_d[1] = constrain(pady_pid->compute(), -v_max, v_max);
        // qd_d = constrain(traj.input[1] * padVars.kp_y, -v_max, v_max);   // velocity control. useful for bringup

        traj.qd_d[2] = constrain(padz_pid->compute(), -w_max, w_max);

        if (pwmReceiver.channels[pwmReceiver.mode_ch]->getCh() > 300){ planner->eStop(); }
        planner->plan_pads(traj.input[0], traj.input[1], traj.input[2], traj.input[3]);
        break;
    }

    int eStopChannel = pwmReceiver.channels[pwmReceiver.estop_ch]->getCh();
    for (int i = 0; i < kin.nWheels; i++) // Send commands to all motors 
    {
      if (modes.mode == 0)
      {
        steer[i]->motTo(planner->getMotAngle(i), pwmReceiver.channels[pwmReceiver.estop_ch]->getCh(), pwmReceiver.rcLost);
        delayMicroseconds(can.steerCanDelay); // Nasty bug where going from 3 motors to 4 per bus required a 100 us delay instead of 50
        drive[i]->slewVel(planner->getDriveWheelSpeed(i), pwmReceiver.channels[pwmReceiver.estop_ch]->getCh(), pwmReceiver.rcLost);
        delayMicroseconds(can.driveCanDelay);
      }
      else
      {
        switch (bringupMode)
        {
        case -1: // 0 = all DOFs simultaneously active
          steer[i]->yawTo(kin.kinematics[i]->getTargetSteer(), eStopChannel, pwmReceiver.rcLost);
          delayMicroseconds(can.driveCanDelay);
          drive[i]->setVel(kin.kinematics[i]->getTargetVel(), eStopChannel, pwmReceiver.rcLost);
          delayMicroseconds(can.driveCanDelay);
          break;
        case 0: // 1 = x axis bringup
          steer[i]->yawTo(180, eStopChannel, pwmReceiver.rcLost);
          delayMicroseconds(can.driveCanDelay);
          drive[i]->setVel(traj.qd_d[0], eStopChannel, pwmReceiver.rcLost);
          delayMicroseconds(can.driveCanDelay);
          break;
        case 1:                                                   // 2 = y axis bringup
          steer[i]->yawTo(-90, eStopChannel, pwmReceiver.rcLost); // should point forwards
          delayMicroseconds(can.driveCanDelay);
          drive[i]->setVel(traj.qd_d[1], eStopChannel, pwmReceiver.rcLost);
          delayMicroseconds(can.driveCanDelay);
          break;
        case 2:                                                             // 3 = z axis bringup
          steer[i]->yawTo(-45 + i * -90, eStopChannel, pwmReceiver.rcLost); // should point CCW
          delayMicroseconds(can.driveCanDelay);
          drive[i]->setVel(traj.qd_d[2], eStopChannel, pwmReceiver.rcLost);
          delayMicroseconds(can.driveCanDelay);
          break;
        default:
          Serial.println("error parsing bringup mode. This is a bug");
          break;
        }
      }
    }
    { // Serial prints @ reduced rate 
    plotCounter++;
    bool plot = plotCounter % 20 == 0;
    if (plot)
    {
      // Serial.print("Mode: ");
      // Serial.println(modes.mode);

      // Serial.print("Target VX:");
      // Serial.print(planner->getTargetVX());
      // Serial.print(" | ");
      // Serial.print("Target VY:");
      // Serial.print(planner->getTargetVY());
      // Serial.print(" | ");      Serial.print("Target VZ:");
      // Serial.println(planner->getTargetVZ());

      Serial.print("Steer0:");
      Serial.print(planner->getSteerAngle(0) * 180/PI);
      Serial.print(" | ");
      Serial.print("Mot0:");
      Serial.print(planner->getMotAngle(0) * 180/PI);
      Serial.print(" | ");
      Serial.print("Drive0:");
      Serial.print(planner->getDriveWheelSpeed(0));
      Serial.print(" | ");
      Serial.print("Temp: ");
      Serial.println(planner->getTemp());
      // pads->printDebug();
    }
    }
  }
  else  // Overtiming 
  {
    loopTiming.behind = false;
  }
  if (loopTiming.now - loopTiming.lastOuter > loopTiming.tOuter)  // Slow loop
  {
    loopTiming.lastOuter = loopTiming.lastOuter + loopTiming.tOuter;
    //***********************BEGIN SLOW LOOP*******************************
    checkRx(); // check if receiver signal has been lost

    // Check channels, modes
    if (pwmReceiver.channels[5]->getCh() > 400)
    {
      calMotor(can); // Calibrate motors
    }
    if (pwmReceiver.channels[pwmReceiver.estop_ch]->getCh() < -400 && pwmReceiver.channels[pwmReceiver.estop_ch]->getCh() < 100)
    { // calibrate force pads, only if steering motoors are off
      // traj.qd_d[0] = 0;
      // traj.qd_d[1] = 0;
      // traj.qd_d[2] = 0;
    }
    if (pwmReceiver.channels[pwmReceiver.mode_ch]->getCh() < -300)
    { // default mode is tele-op, blue stick top position
      if (modes.mode != 0)
      {
        Serial.println("Mode 0 entered (tele-op)");
      }
      modes.mode = 0;
      planner->setMode(0);
      modes.eStop = 0; // Also disable e-stop if tripped
      modes.zeroing = 0;
    }
    if (pwmReceiver.channels[pwmReceiver.mode_ch]->getCh() > -300 && pwmReceiver.channels[pwmReceiver.mode_ch]->getCh() < 300)
    { // IMU zeroing mode
      if (modes.mode != 2)
      {
        Serial.println("Mode 2 entered");
        modes.mode = 2;
        delay(100);
      }
      // planner->setMode(2);
      modes.zeroing = 0;
    }
    if (pwmReceiver.channels[pwmReceiver.mode_ch]->getCh() > 300)
    { // riding mode, blue stick down position
      if (!modes.zeroing)
      {
        Serial.println("Zeroing entered");
      }
      modes.zeroing = 1;
      switch (modes.mode)
      {
      case 1:
        Serial.println("IMU zeroing not supported");
        break;
      case 2:
        pads->calibrate();
        traj.qd_d[0] = 0;
        traj.qd_d[1] = 0;
        traj.qd_d[2] = 0;
        Serial.println("Pads zeroing");
        break;
      default:
        Serial.println("Unable to parse mode to enter zeroing. This is probably a bug");
      }
    }

    // Debug related
    if (modes.debugRiding)
    {
      Serial.println("***********");
      Serial.println(traj.qd_d[0]);
      Serial.println(traj.qd_d[1]);
      delay(20);
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
    steer[j]->motTo(0, pwmReceiver.channels[pwmReceiver.estop_ch]->getCh(), pwmReceiver.rcLost);
  }
  delay(1000);          // Give motor time to move to zero position if it is wound up
  double fineTune = 1.0; // to step in less than 1 deg increments - this is the ratio (0.2 would be in 0.2 degree increments
  for (int i = 0; i < (int)(360 * abs(kin.yRatio) * 1.5 / fineTune); i++)
  {
    checkRx();
    for (int j = 0; j < kin.nWheels; j++)
    {
      can.pos = i * fineTune;
      if (steer[j]->getHoming() == 2 && digitalRead(robotState.irPin[j]) == 1)
      { // calibration started with IR triggered
        steer[j]->setHoming(1);
      }
      if (steer[j]->getHoming() == 1 && digitalRead(robotState.irPin[j]) == 0)
      { // Hit target fresh
        steer[j]->setYaw(robotState.irPos[j]);
        steer[j]->setMPos(can.pos);
        steer[j]->setHoming(0);
      }
      if (steer[j]->getHoming() == 0)
      { // position signal should persist, but motor should stop moving after cal marker is detected
        can.pos = steer[j]->getMPos();
      }
      steer[j]->motTo(can.pos * PI / 180.0, pwmReceiver.channels[pwmReceiver.estop_ch]->getCh(), pwmReceiver.rcLost);
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
// This function detects if a receiver signal has been received recently. Used for safety, etc.
void checkRx()
{
  if (micros() - pwmReceiver.channels[pwmReceiver.estop_ch]->getLastInterruptTime() > pwmReceiver.rcTimeout)
  {
    pwmReceiver.rcLost = 1;
  }
  else
  {
    pwmReceiver.rcLost = 0;
  }
}
void telemetry()
{
  unsigned long time = micros();
  if (time - prevTelemetryReportTime < TELEMETRY_REPORT_PERIOD)
  {
    return;
  }
  prevTelemetryReportTime = time;

  Serial.println("\n");
}
// 2.4 GHz RECEIVER  FUNCTIONS
void calcCh1()
{
  pwmReceiver.channels[0]->calc();
}
void calcCh2()
{
  pwmReceiver.channels[1]->calc();
}
void calcCh3()
{
  pwmReceiver.channels[2]->calc();
}
void calcCh4()
{
  pwmReceiver.channels[3]->calc();
}
void calcCh5()
{
  pwmReceiver.channels[4]->calc();
}
void calcCh6()
{
  pwmReceiver.channels[5]->calc();
}
void calcCh7()
{
  pwmReceiver.channels[6]->calc();
}
void calcCh8()
{
  pwmReceiver.channels[7]->calc();
}
#pragma endregion