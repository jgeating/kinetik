#include "DueCANLayer.h";      // CAN library for CAN shield
#include <math.h>;             // Math functions
#include "Kinematics.h";       // wheel level kinematics/trigonometry
#include "Planner.h";          // robot level planning
#include "utils.h";            // Basic utils like more powerful serial
#include "Yaw.h";              // For controlling steering actuator
#include "Pads.h";             // For interfacing with weight pads
#include "Drive.h";            // For controlling drive motors
#include <Wire.h>;             // For accessing native Arduino I2C functions
#include "Adafruit_Sensor.h"   // Downloaded library for IMU stuff
#include "Adafruit_BNO055.h"   // Downloaded library for IMU stuff
#include "utility/imumaths.h"  // Downloaded library for IMU stuff
#include "Swerve.h";
#include "Performance.h";
#include "PID.h";              // For PID controllers
#include "LowPassFilter.cpp";  // Low pass filter class

const int CHANNEL_PIN[] = {
  38,  // left stick vertical, forward = (+)
  40,  // right stick horizontal, R = (+)
  42,  // right stick vertical, forward = (+)
  44,  // left stick horizontal, R = (+)
  46,  // left knob, CW = (+)
  48,  // right knob, CW = (+)
  50,  // left switch, backwards = (-)
  52   // right switch, backwards = (-)
};

#define RADIUS_SWERVE_ASSEMBLY 0.25  // distance to wheel swerve axes, meters
#define DEAD_ZONE 0.1
#define pi 3.14159265358979
#define BNO055_SAMPLERATE_DELAY_MS (10)
#define TELEMETRY_REPORT_PERIOD 500000

// General stuff, controls 
char buff[100] = "";  // For various sprintf print outs
double qd_d = 0;    // temporary used for testing vest acceleration control. Stores instantaneous velocity
double qdd_d = 0;
double temp = 0;      // generic doubles for testing new things
double temp2 = 0;
double temp3 = 0;
double dt = 0;        // loop period. Pulled from timing loop parameter, converted from usec to sec
int bringupMode = -1;  // Bringup mode. set to -1 for normal operation, 0... are for bringing up specific axes for single DOF PID tuning 0 = X, 1 = Y, 1 = Z

// IMU stuff
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29);      // (id, address), default 0x29 or 0x28
Adafruit_BNO055 bnoVest = Adafruit_BNO055(-1, 0x28);  // (id, address), default 0x29 or 0x28
double vestAngleCenter = 0;
bool vestIMUsetup = 0;
bool foc = false;  // Whether or not to drive in field oriented control. True = robot IMU used
double alpha = 0;  // Angle used for calculating field oriented control 
imu::Vector<3> euler;      // Orientation of robot
imu::Vector<3> gyroRobot;  // Rotation rate of robot (raw gyro signal)
imu::Vector<3> eulerVest;  // Euler orientation of vest
imu::Vector<3> gyroVest;   // Rotation rate of vest (raw gyro signal)

//Instantiate structs
SwerveTrajectory swerveTrajectory;
imu_vars imuVars;
vest_vars vestVars;
pad_vars padVars;
SwerveKinematics swerveKinematics;
LoopTiming loopTiming; 
Profiles profiles;  
Modes modes;  // Handles different driving and control modes

// Instantiate classes? 
Planner *planner; // for planning robot level motion
Pads *pads;  // For driving with force pads
PID *padx_pid;
PID *pady_pid;
PID *padz_pid;

// CAN Stuff for drive motors
extern byte canInit(byte cPort, long lBaudRate);
extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte *cData, byte cDataLen);
extern byte canRx(byte cPort, long *lMsgID, bool *bExtendedFormat, byte *cData, byte *cDataLen);
SwerveCAN can;

// PWM/Receiver stuff
PWMReceiver pwmReceiver;  
RobotState robotState;
Drive **drive = new Drive *[swerveKinematics.nWheels];
Yaw **yaw = new Yaw *[swerveKinematics.nWheels];
double aMaxYaw = 5000;   // Max angular acceleration of yaw motor, in motor frame, rad/s^2. Safe starting value: 5000
double wMaxYaw = 10000;  // Max angular velocity of yaw motor, in motor frame, rad/s. Safe starting value: 10000
int doneHoming = 0;      // Used to determine when calibration sequence is finished. 1 = finished.

// Plotting, telemetry
int plotCounter = 0;
unsigned long prevTelemetryReportTime = 0;
Watchdog watchdog;
LowPassFilter filter(10);  // create a low-pass filter with 10 readings

void setup() {
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
  for (int i = 0; i < 4; i++) {
    pinMode(robotState.irPin[i], INPUT);
  }
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);  // Set up indicator LED
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);  // Set address of Vest IMU to high
  Serial.println("CAN and Pins initialized. Initializing RC interrupts...");

  // RC PWM interrupt setup
  for (int i = 0; i < pwmReceiver.chs; i++) {
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
  Serial.println("RC interrrupts initialized. Setting up kinematics and trajectory planning objects...");

  // Robot IMU setup
  setupImu("robot IMU", bno);
  bno.setExtCrystalUse(true);
  delay(100);
  Serial.println("Done setting up robot IMU");

  // Kinematics and path planning setup
  swerveKinematics.dRatio = swerveKinematics.pole_pairs * 60 / (2 * M_PI) / (.083 / 2);  // used to convert m/s to rpm
  for (int i = 0; i < swerveKinematics.nWheels; i++) {
    robotState.irPos[i] = robotState.irPos[i];  // Correcting for polar coordinate frame
    yaw[i] = new Yaw(wMaxYaw, aMaxYaw, robotState.yRatio, loopTiming.tInner, can.len, i);
    drive[i] = new Drive(swerveTrajectory.qd_max[0], swerveTrajectory.qdd_max[0], swerveKinematics.dRatio, loopTiming.tInner, can.len, i);
    swerveKinematics.kinematics[i] = new Kinematics(RADIUS_SWERVE_ASSEMBLY, DEAD_ZONE, i);
  }
  planner = new Planner(loopTiming.tInner, swerveTrajectory.qd_max[0], swerveTrajectory.qd_max[1], swerveTrajectory.qd_max[2], swerveTrajectory.qdd_max[0], swerveTrajectory.qdd_max[1], swerveTrajectory.qdd_max[2], swerveTrajectory.dz[0], swerveTrajectory.dz[1], swerveTrajectory.dz[2], modes.mode, vestVars.maxLean);

  // Set up pad pid classes
  dt = loopTiming.tInner/1000000.0;
  pads = new Pads();
  padx_pid = new PID(padVars.kp_x, padVars.ki_x, padVars.kd_x, dt, padVars.lag_x);
  pady_pid = new PID(padVars.kp_y, padVars.ki_y, padVars.kd_y, dt, padVars.lag_y);
  padz_pid = new PID(padVars.kp_z, padVars.ki_z, padVars.kd_z, dt, padVars.lag_z);
  Serial.println("Startup Complete.");
}

void loop() {
  startProfile(profiles.robotLoop);
  // printWatchdogError(watchdog);
  // telemetry();
  loopTiming.now = micros();
  if (loopTiming.now - loopTiming.lastInner > loopTiming.tInner) {
    startProfile(profiles.loopTiming);
    if (loopTiming.behind) {  // This means we can't keep up with the desired loop rate. Trip LED to indicate so
      digitalWrite(13, HIGH);
      loopTiming.lastInner = loopTiming.now;
    } else {
      digitalWrite(13, LOW);
      loopTiming.lastInner = loopTiming.lastInner + loopTiming.tInner;
      loopTiming.behind = true;
    }
    endProfile(profiles.loopTiming);
    //*************************************************
    //***************BEGIN FAST LOOP*******************
    //*************************************************
    if (modes.mode == 0) {  // 0 = tele-op
      startProfile(profiles.mode0);

      // ******************* tele-op mode ************************
      // PWM conditioning to send to planner
      double global_gain = float(constrain(pwmReceiver.channels[0]->getCh(), -500, 500) / 1000.0 + 0.5);  // Maps to 0.0 - 1.0 
      for (int k = 0; k < 3; k++) {
        swerveTrajectory.input[k] = constrain(pwmReceiver.channels[k + 1]->getCh() / 500.0, -1.0, 1.0);
        swerveTrajectory.input[k] = swerveTrajectory.input[k] * swerveTrajectory.qd_max[k];
        if (k < 2) {
          swerveTrajectory.input[k] = swerveTrajectory.input[k] * global_gain;  // Scale up to max velocity using left stick vertical (throttle, no spring center)
        } else {
          swerveTrajectory.input[k] = swerveTrajectory.input[k] * (global_gain + 1.0) / 2.0;  // for yaw, only scale between 50% and 100%, not 0 and 100%
        }
      }
      if (foc == false) {
        planner->plan(swerveTrajectory.input[0], swerveTrajectory.input[1], -swerveTrajectory.input[2]);
      } else {
        loopTiming.timer[1] = micros();
        euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        alpha = euler.x() * pi / 180.0;
        loopTiming.timer[2] = micros();
        planner->plan_world(swerveTrajectory.input[0], swerveTrajectory.input[1], -swerveTrajectory.input[2], alpha);
      }

      // Get results from planner
      swerveTrajectory.qd_d[0] = planner->getTargetVX();
      swerveTrajectory.qd_d[1] = planner->getTargetVY();
      swerveTrajectory.qd_d[2] = planner->getTargetVZ();
      endProfile(profiles.mode0);

      // *************************** riding modes ************************
    } else if (modes.mode > 0) { // mode 0 = tele-op. all other modes > 0 (IMU = 1, force pads = 2)
      startProfile(profiles.modeOther);
      if (modes.mode == 1) {  // IMU mode
        if (vestIMUsetup == false) {
          setupImu("robot Vest", bnoVest);
          vestIMUsetup = true;
          Serial.println("done setting up vest IMU");
        }
        eulerVest = bnoVest.getVector(Adafruit_BNO055::VECTOR_EULER);
        vestVars.x = eulerVest.z() * M_PI / 180;  // forward/backward lean angle
        vestVars.y = eulerVest.y() * M_PI / 180;  // left/right lean angle
        vestVars.z = eulerVest.x() * M_PI / 180;  // yaw rotation angle
        gyroVest = bnoVest.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        vestVars.xRate = gyroVest.z();  // forward/backward lean rate
        vestVars.yRate = gyroVest.y();  // left/right lean rate
        vestVars.zRate = gyroVest.x();  // yaw rate

        euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        imuVars.zRobot = euler.x() * M_PI / 180;  // robot base yaw
        gyroRobot = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        imuVars.zRobotRate = gyroRobot.x();

        // Set controller inputs
        swerveTrajectory.input[0] = vestVars.kp_x * (vestVars.x - vestVars.xZero);  // calculate proportional term
        swerveTrajectory.input[1] = vestVars.kp_y * (vestVars.y - vestVars.yZero);
        double del_th = (vestVars.z - vestVars.zZero) - (imuVars.zRobot - imuVars.zZero_robot);
        while (abs(del_th) > M_PI / 2) {
          del_th = del_th - sign(del_th) * (M_PI * 2);
        }
        swerveTrajectory.input[2] = vestVars.kp_z * del_th;               // calculate proportional term
      }

      if (modes.mode == 2){      // Begin force pad mode
        double a_max = 15;      //limit max acceleration, m/s^2
        double v_max = 10;      //limit max velocity
        double alpha_max = 20;  // limit max angular acceleration, rad/s^2
        double w_max = 30;      // limit max angular velocity, rad/s

        // swerveTrajectory.input[0] = 0;
        // swerveTrajectory.input[1] = 0;
        // swerveTrajectory.input[2] = 0;

        pads->calcVector(); 
        swerveTrajectory.input[0] = pads->getX();
        swerveTrajectory.input[1] = pads->getY();
        swerveTrajectory.input[2] = pads->getZ();

        qdd_d = swerveTrajectory.input[0] * padVars.kp_x;
        qdd_d = constrain(qdd_d, -a_max, a_max);
        qd_d = swerveTrajectory.qd_d[0] + qdd_d * dt;
        qd_d = constrain(qd_d, -v_max, v_max);
        // qd_d = constrain(swerveTrajectory.input[0] * padVars.kp_x, -v_max, v_max);   // velocity control. useful for bringup
        swerveTrajectory.qd_d[0] = qd_d;

        qdd_d = swerveTrajectory.input[1] * padVars.kp_y;
        qdd_d = constrain(qdd_d, -a_max, a_max);
        qd_d = swerveTrajectory.qd_d[1] + qdd_d * dt;
        qd_d = constrain(qd_d, -v_max, v_max);
        // qd_d = constrain(swerveTrajectory.input[1] * padVars.kp_y, -v_max, v_max);   // velocity control. useful for bringup
        swerveTrajectory.qd_d[1] = qd_d;

        qd_d = swerveTrajectory.input[2] * padVars.kp_z;
        qd_d = constrain(qd_d, -w_max, w_max);
        swerveTrajectory.qd_d[2] = qd_d;

        // padx_pid->setInput(swerveTrajectory.qd_d[0]);  // Give current velocity as input
        // pady_pid->setInput(swerveTrajectory.qd_d[1]);
        // padz_pid->setInput(swerveTrajectory.qd_d[2]);

        // padx_pid->setSetpoint(swerveTrajectory.input[0]);
        // pady_pid->setSetpoint(swerveTrajectory.input[1]);
        // padz_pid->setSetpoint(swerveTrajectory.input[2]);

        // swerveTrajectory.qd_d[0] = constrain(padx_pid->compute(), -v_max, v_max);
        // swerveTrajectory.qd_d[1] = constrain(pady_pid->compute(), -v_max, v_max);
        // swerveTrajectory.qd_d[2] = constrain(padz_pid->compute(), -w_max, w_max);
      }

      if (pwmReceiver.channels[pwmReceiver.mode_ch]->getCh() > 300) { // In an estop, immediately set velocities to zero, for now. May implement deceleration in the future 
        swerveTrajectory.qd_d[0] = 0;
        swerveTrajectory.qd_d[1] = 0;
        swerveTrajectory.qd_d[2] = 0;
      }

      // for (int i = 0; i < 3; i++) {
      //   swerveTrajectory.input[i] = swerveTrajectory.input[i];  // Convert inputs to radians
      // }
      // planner->plan(swerveTrajectory.input[0], swerveTrajectory.input[1], swerveTrajectory.input[2]);
      endProfile(profiles.modeOther);
    }

    startProfile(profiles.kinematics);

    for (int i = 0; i < swerveKinematics.nWheels; i++) {
      if (modes.mode == 0){ // tele-op
        swerveKinematics.kinematics[i]->calc(swerveTrajectory.qd_d[0], swerveTrajectory.qd_d[1], swerveTrajectory.qd_d[2] * sign(robotState.yRatio));
      } else {  // IMU riding 
        swerveKinematics.kinematics[i]->calc(swerveTrajectory.qd_d[0], swerveTrajectory.qd_d[1], swerveTrajectory.qd_d[2] * sign(robotState.yRatio));
      }
    }
    endProfile(profiles.kinematics);

    startProfile(profiles.updateMotorSpeeds);
    int eStopChannel = pwmReceiver.channels[pwmReceiver.estop_ch]->getCh();
    bool teleop = pwmReceiver.channels[pwmReceiver.mode_ch]->getCh() < -300;

    if (modes.mode > 0) {
      // double delVestRobot = (vestVars.z - vestVars.zZero) - (imuVars.zRobot - imuVars.zZero_robot);
      // while (abs(delVestRobot) > 180) { // unwrap to keep within +/- 180 deg
      //   delVestRobot = delVestRobot - sign(delVestRobot) * 360.0;
      // }
      // v_temp = delVestRobot/-5.0;
    }

    for (int i = 0; i < swerveKinematics.nWheels; i++) {
      if (modes.mode == 0) {
        yaw[i]->yawTo(swerveKinematics.kinematics[i]->getTargetYaw(), pwmReceiver.channels[pwmReceiver.estop_ch]->getCh(), pwmReceiver.rcLost);
        delayMicroseconds(can.steerCanDelay);  // Nasty bug where going from 3 motors to 4 per bus required a 100 us delay instead of 50
        // drive[i]->slewVel(pwmReceiver.channels[0]->getCh() / 100.0, pwmReceiver.channels[pwmReceiver.estop_ch]->getCh(), pwmReceiver.rcLost);
        drive[i]->slewVel(swerveKinematics.kinematics[i]->getTargetVel(), pwmReceiver.channels[pwmReceiver.estop_ch]->getCh(), pwmReceiver.rcLost);
        delayMicroseconds(can.driveCanDelay);
      } else {
        switch (bringupMode) {
          case -1:   // 0 = all DOFs simultaneously active
            yaw[i]->yawTo(swerveKinematics.kinematics[i]->getTargetYaw(), eStopChannel, pwmReceiver.rcLost);
            delayMicroseconds(can.driveCanDelay);
            drive[i]->setVel(swerveKinematics.kinematics[i]->getTargetVel(), eStopChannel, pwmReceiver.rcLost);
            delayMicroseconds(can.driveCanDelay);
            break;
          case 0:   // 1 = x axis bringup
            yaw[i]->yawTo(0, eStopChannel, pwmReceiver.rcLost);
            delayMicroseconds(can.driveCanDelay);
            drive[i]->setVel(swerveTrajectory.qd_d[0], eStopChannel, pwmReceiver.rcLost);
            delayMicroseconds(can.driveCanDelay);
            break;
          case 1: // 2 = y axis bringup
            yaw[i]->yawTo(-90, eStopChannel, pwmReceiver.rcLost);  // should point forwards
            delayMicroseconds(can.driveCanDelay);
            drive[i]->setVel(swerveTrajectory.qd_d[1], eStopChannel, pwmReceiver.rcLost);
            delayMicroseconds(can.driveCanDelay);
            break;
          case 2: // 3 = z axis bringup
            yaw[i]->yawTo(-45 + i * -90, eStopChannel, pwmReceiver.rcLost); // should point CCW
            delayMicroseconds(can.driveCanDelay);
            drive[i]->setVel(swerveTrajectory.qd_d[2], eStopChannel, pwmReceiver.rcLost);
            delayMicroseconds(can.driveCanDelay);
            break;
          default: 
            Serial.println("error parsing bringup mode. This is a bug");
            break;
        }
        
      }
    }

    plotCounter++;
    bool plot = plotCounter % 20 == 0;
    if (plot) {
      // Serial.print("Mode: ");
      // Serial.println(modes.mode);
      // Serial.print("X_d:");
      // Serial.print(swerveTrajectory.input[0]);
      // Serial.print(",");
      // Serial.print("Y_d:");
      // Serial.print(swerveTrajectory.input[1]);
      // Serial.print(",");
      // Serial.print("Z_d:");
      // Serial.println(swerveTrajectory.input[2]);
      // pads->printDebug();
    }
    endProfile(profiles.updateMotorSpeeds);
  } else {
    loopTiming.behind = false;
  }

  startProfile(profiles.outerLoop);
  if (loopTiming.now - loopTiming.lastOuter > loopTiming.tOuter) {
    loopTiming.lastOuter = loopTiming.lastOuter + loopTiming.tOuter;
    //***********************BEGIN SLOW LOOP*******************************
    checkRx();  // check if receiver signal has been lost

    // Check channels, modes
    if (pwmReceiver.channels[5]->getCh() > 400) {
      calMotor(can);  // Calibrate motors
    }
    if (pwmReceiver.channels[pwmReceiver.estop_ch]->getCh() < -400 && pwmReceiver.channels[pwmReceiver.estop_ch]->getCh() < 100) {  // calibrate force pads, only if steering motoors are off
      // swerveTrajectory.qd_d[0] = 0;
      // swerveTrajectory.qd_d[1] = 0;
      // swerveTrajectory.qd_d[2] = 0;
    }
    if (pwmReceiver.channels[pwmReceiver.mode_ch]->getCh() < -300) {  // default mode is tele-op, blue stick top position
      if (modes.mode != 0) {
        Serial.println("Mode 0 entered (tele-op)");
      }
      modes.mode = 0;
      planner->setMode(0);
      modes.eStop = 0;  // Also disable e-stop if tripped
      modes.zeroing = 0;
    }
    if (pwmReceiver.channels[pwmReceiver.mode_ch]->getCh() > -300 && pwmReceiver.channels[pwmReceiver.mode_ch]->getCh() < 300) {  // IMU zeroing mode
      if (modes.mode != 2) {
        Serial.println("Mode 2 entered");
        modes.mode = 2;
        delay(100);
      }
      // planner->setMode(2);
      modes.zeroing = 0;
    }
    if (pwmReceiver.channels[pwmReceiver.mode_ch]->getCh() > 300) {  // riding mode, blue stick down position
      if (!modes.zeroing) {
        Serial.println("Zeroing entered");
      }
      modes.zeroing = 1;
      switch (modes.mode){
        case 1: 
          zeroImus();
          Serial.println("IMU zeroing");
          break;
        case 2: 
          pads->calibrate();
          swerveTrajectory.qd_d[0] = 0;
          swerveTrajectory.qd_d[1] = 0;
          swerveTrajectory.qd_d[2] = 0;
          Serial.println("Pads zeroing");
          break;
        default: 
          Serial.println("Unable to parse mode to enter zeroing. This is probably a bug");
      }
    }

    // Debug related
    if (modes.debugRiding) {
      Serial.println("***********");
      Serial.println(swerveTrajectory.qd_d[0]);
      Serial.println(swerveTrajectory.qd_d[1]);
      delay(20);
    }
  }
  endProfile(profiles.outerLoop);

  endProfile(profiles.robotLoop);
  // printProfiles(profiles);
}

// *********************************************** HELPER FUNCTIONS **************************************************************

// ************** CALIBRATION **********************
void calMotor(SwerveCAN &can) {
  for (int j = 0; j < swerveKinematics.nWheels; j++) {
    yaw[j]->setHoming(2);  // set homing mode to true for all axes
    yaw[j]->motTo(0, pwmReceiver.channels[pwmReceiver.estop_ch]->getCh(), pwmReceiver.rcLost);
  }
  delay(750);           // Give motor time to move to zero position if it is wound up
  double fineTune = 1;  // to step in less than 1 deg increments - this is the ratio (0.2 would be in 0.2 degree increments
  double angTarget = 0;
  for (int i = 0; i < (int)(360 * abs(robotState.yRatio) * 1.5 / fineTune); i++) {
    checkRx();
    for (int j = 0; j < swerveKinematics.nWheels; j++) {
      angTarget = i * fineTune;
      can.pos = i * fineTune;
      if (yaw[j]->getHoming() == 2 && digitalRead(robotState.irPin[j]) == 1) {  // calibration started with IR triggered
        yaw[j]->setHoming(1);
      }
      if (yaw[j]->getHoming() == 1 && digitalRead(robotState.irPin[j]) == 0) {  // Hit target fresh
        yaw[j]->setYaw(robotState.irPos[j]);
        yaw[j]->setMPos(can.pos);
        yaw[j]->setHoming(0);
      }
      if (yaw[j]->getHoming() == 0) {  // position signal should persist, but motor should stop moving after cal marker is detected
        can.pos = yaw[j]->getMPos();
      }
      yaw[j]->motTo(can.pos, pwmReceiver.channels[pwmReceiver.estop_ch]->getCh(), pwmReceiver.rcLost);
    }
    delayMicroseconds(800);
    doneHoming = 1;
    for (int j = 0; j < swerveKinematics.nWheels; j++) {
      doneHoming = doneHoming && yaw[j]->getHoming() == 0;
    }
    if (doneHoming == 1) {
      swerveKinematics.calibrated = 1;  // set calibration flag to true
    }
    if (doneHoming == 1)
      // while (1){}  % used to freeze homing positions after cal to measure offsets with phone/level
      break;  // break from loop once all targets have been found
  }
}

double mapDouble(double x, double min_in, double max_in, double min_out, double max_out) {
  double ret = (x - min_in) / (max_in - min_in);
  ret = ret * (max_out - min_out) + min_out;
  return ret;
}

void zeroImus() {
  vestVars.xZero = vestVars.x;
  vestVars.yZero = vestVars.y;
  vestVars.zZero = vestVars.z;
  imuVars.zZero_robot = imuVars.zRobot;
  // Serial.println("Zeroing IMU's");
  // char buffer[100] = "";
  // sprintf(buffer, "Vest X: %.3f, Y: %.3f, Z: %.3f | Robot Z: %.3f", vestVars.x, vestVars.y, vestVars.z, imuVars.zrobot);
  // Serial.println(buffer);
  qd_d = 0;  // reset instantaneous velocity to zero for acceleration control
}

// This function detects if a receiver signal has been received recently. Used for safety, etc.
void checkRx() {
  if (micros() - pwmReceiver.channels[pwmReceiver.estop_ch]->getLastInterruptTime() > pwmReceiver.rcTimeout) {
    pwmReceiver.rcLost = 1;
  } else {
    pwmReceiver.rcLost = 0;
  }
}

// ************************* CAN RECEIVER
// void rxMsg()
//{
//  if (canRx(0, &can.lMsgID, &can.bExtendedFormat, &can.cRxData[0], &can.cDataLen) == CAN_OK)
//  {
//    can.rxData = 0;
//    for (int i = 0; i < can.len; i++)
//    {
//      can.rxData = can.rxData << 8;
//      can.rxData |= can.cRxData[i];
//    }
//  }
//}

void telemetry() {
  unsigned long time = micros();
  if (time - prevTelemetryReportTime < TELEMETRY_REPORT_PERIOD) {
    return;
  }
  prevTelemetryReportTime = time;

  Serial.println("\n");
  printImu("Robot IMU", bno);
  // printImu("Vest IMU", bnoVest);
}

void setupImu(String name, Adafruit_BNO055 &imu) {
  Serial.println("Setting up " + name + "...");
  if (!imu.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  } else {
    Serial.println("Successfully connected to IMU");
  }
  delay(100);
}

void printImu(String name, Adafruit_BNO055 &imu) {
  imu::Vector<3> euler = imu.getVector(Adafruit_BNO055::VECTOR_EULER);
  uint8_t *system_status;
  uint8_t *self_test_result;
  uint8_t *system_error;
  imu.getSystemStatus(system_status, self_test_result, system_error);
  Serial.print(name + ": ");
  serialPrintln(100, "(%.2f, %.2f, %.2f)", euler.x(), euler.y(), euler.z());
  Serial.print(name + " status:");
  Serial.println(*self_test_result);
}

// ***********************2.4 GHz RECEIVER  FUNCTIONS
void calcCh1() {
  pwmReceiver.channels[0]->calc();
}
void calcCh2() {
  pwmReceiver.channels[1]->calc();
}
void calcCh3() {
  pwmReceiver.channels[2]->calc();
}
void calcCh4() {
  pwmReceiver.channels[3]->calc();
}
void calcCh5() {
  pwmReceiver.channels[4]->calc();
}
void calcCh6() {
  pwmReceiver.channels[5]->calc();
}
void calcCh7() {
  pwmReceiver.channels[6]->calc();
}
void calcCh8() {
  pwmReceiver.channels[7]->calc();
}
