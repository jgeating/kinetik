#include "DueCANLayer.h";      // CAN library for CAN shield
#include <math.h>;             // Math functions
#include "Kinematics.h";       // wheel level kinematics/trigonometry
#include "Planner.h";          // robot level planning
#include "shared/utils.h";            // Basic utils like more powerful serial
#include "penny/Steer.h";              // For controlling steering actuator
#include "penny/Pads.h";             // For interfacing with weight pads
#include "penny/Drive.h";            // For controlling drive motors
#include <Wire.h>;             // For accessing native Arduino I2C functions
#include "Adafruit_Sensor.h"   // Downloaded library for IMU stuff
#include "Adafruit_BNO055.h"   // Downloaded library for IMU stuff
#include "utility/imumaths.h"  // Downloaded library for IMU stuff
#include "Swerve.h";
#include "Performance.h";
#include "PID.h";              // For PID controllers
#include "shared/LowPassFilter.cpp";  // Low pass filter class

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
int bringupMode = 0;  // Bringup mode. set to -1 for normal operation, 0... are for bringing up specific axes for single DOF PID tuning 0 = X, 1 = Y, 1 = Z

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
SwerveTrajectory traj;
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
Steer **steer = new Steer *[swerveKinematics.nWheels];
double aMaxSteer = 5000;   // Max angular acceleration of steer motor, in motor frame, rad/s^2. Safe starting value: 5000
double wMaxSteer = 10000;  // Max angular velocity of steer motor, in motor frame, rad/s. Safe starting value: 10000
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
    steer[i] = new Steer(wMaxSteer, aMaxSteer, robotState.yRatio, loopTiming.tInner, can.len, i);
    drive[i] = new Drive(traj.qd_max[0], traj.qdd_max[0], swerveKinematics.dRatio, loopTiming.tInner, can.len, i);
    swerveKinematics.kinematics[i] = new Kinematics(RADIUS_SWERVE_ASSEMBLY, DEAD_ZONE, i);
  }
  planner = new Planner(loopTiming.tInner, traj.qd_max[0], traj.qd_max[1], traj.qd_max[2], traj.qdd_max[0], traj.qdd_max[1], traj.qdd_max[2], traj.dz[0], traj.dz[1], traj.dz[2], modes.mode, vestVars.maxLean);

  // Set up pad pid classes
  dt = loopTiming.tInner/1000000.0;
  pads = new Pads();
  padx_pid = new PID(padVars.kp_x, padVars.ki_x, padVars.kd_x, dt, padVars.lag_x);
  pady_pid = new PID(padVars.kp_y, padVars.ki_y, padVars.kd_y, dt, padVars.lag_y);
  padz_pid = new PID(padVars.kp_z, padVars.ki_z, padVars.kd_z, dt, padVars.lag_z);

  padx_pid->setSetpoint(0);   // setpoint = 0 means try to put human center of pressure at middle of footpad 
  pady_pid->setSetpoint(0);
  padz_pid->setSetpoint(0);

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
        traj.input[k] = constrain(pwmReceiver.channels[k + 1]->getCh() / 500.0, -1.0, 1.0);
        traj.input[k] = traj.input[k] * traj.qd_max[k];
        if (k < 2) {
          traj.input[k] = traj.input[k] * global_gain;  // Scale up to max velocity using left stick vertical (throttle, no spring center)
        } else {
          traj.input[k] = traj.input[k] * (global_gain + 1.0) / 2.0;  // for steer, only scale between 50% and 100%, not 0 and 100%
        }
      }
      if (foc == false) {
        planner->plan(traj.input[0], traj.input[1], -traj.input[2]);
      } else {
        loopTiming.timer[1] = micros();
        euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        alpha = euler.x() * pi / 180.0;
        loopTiming.timer[2] = micros();
        planner->plan_world(traj.input[0], traj.input[1], -traj.input[2], alpha);
      }

      // Get results from planner
      traj.qd_d[0] = planner->getTargetVX();
      traj.qd_d[1] = planner->getTargetVY();
      traj.qd_d[2] = planner->getTargetVZ();
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
        vestVars.z = eulerVest.x() * M_PI / 180;  // steer rotation angle
        gyroVest = bnoVest.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        vestVars.xRate = gyroVest.z();  // forward/backward lean rate
        vestVars.yRate = gyroVest.y();  // left/right lean rate
        vestVars.zRate = gyroVest.x();  // steer rate

        euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        imuVars.zRobot = euler.x() * M_PI / 180;  // robot base steer
        gyroRobot = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        imuVars.zRobotRate = gyroRobot.x();

        // Set controller inputs
        traj.input[0] = vestVars.kp_x * (vestVars.x - vestVars.xZero);  // calculate proportional term
        traj.input[1] = vestVars.kp_y * (vestVars.y - vestVars.yZero);
        double del_th = (vestVars.z - vestVars.zZero) - (imuVars.zRobot - imuVars.zZero_robot);
        while (abs(del_th) > M_PI / 2) {
          del_th = del_th - sign(del_th) * (M_PI * 2);
        }
        traj.input[2] = vestVars.kp_z * del_th;               // calculate proportional term
      }

      if (modes.mode == 2){      // Begin force pad mode
        double a_max = 15;      //limit max acceleration, m/s^2
        double v_max = 10;      //limit max velocity
        double alpha_max = 20;  // limit max angular acceleration, rad/s^2
        double w_max = 30;      // limit max angular velocity, rad/s

        pads->calcVector(); 
        padx_pid->setInput(pads->getX());
        pady_pid->setInput(pads->getY());
        padz_pid->setInput(pads->getZ());

        // qdd_d = traj.input[0] * padVars.kp_x;    // driving without PID controller, just P controller
        // qdd_d = constrain(qdd_d, -a_max, a_max);
        traj.qdd_d[0] = constrain(padx_pid->compute(), -a_max, a_max);
        traj.qd_d[0] = constrain(traj.qd_d[0] + traj.qdd_d[0] * dt, -v_max, v_max);
        // qd_d = constrain(traj.input[0] * padVars.kp_x, -v_max, v_max);   // velocity control. useful for bringup

        // qdd_d = traj.input[1] * padVars.kp_y;    // driving without PID controller, just P controller
        // qdd_d = constrain(qdd_d, -a_max, a_max);

        // traj.qdd_d[1] = constrain(pady_pid->compute(), -a_max, a_max);
        // traj.qd_d[1] = constrain(traj.qd_d[1] + traj.qdd_d[1] * dt, -v_max, v_max);
        traj.qd_d[1] = constrain(pady_pid->compute(), -v_max, v_max);
        // qd_d = constrain(traj.input[1] * padVars.kp_y, -v_max, v_max);   // velocity control. useful for bringup


        traj.qd_d[2] = constrain(padz_pid->compute(), -w_max, w_max);
      }

      if (pwmReceiver.channels[pwmReceiver.mode_ch]->getCh() > 300) { // In an estop, immediately set velocities to zero, for now. May implement deceleration in the future 
        traj.qd_d[0] = 0;
        traj.qd_d[1] = 0;
        traj.qd_d[2] = 0;
      }
      // planner->plan(traj.input[0], traj.input[1], traj.input[2]);
      endProfile(profiles.modeOther);
    }

    startProfile(profiles.kinematics);

    for (int i = 0; i < swerveKinematics.nWheels; i++) {
      if (modes.mode == 0){ // tele-op
        swerveKinematics.kinematics[i]->calc(traj.qd_d[0], traj.qd_d[1], traj.qd_d[2] * sign(robotState.yRatio));
      } else {  // IMU riding 
        swerveKinematics.kinematics[i]->calc(traj.qd_d[0], traj.qd_d[1], traj.qd_d[2] * sign(robotState.yRatio));
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
        steer[i]->yawTo(swerveKinematics.kinematics[i]->getTargetSteer(), pwmReceiver.channels[pwmReceiver.estop_ch]->getCh(), pwmReceiver.rcLost);
        delayMicroseconds(can.steerCanDelay);  // Nasty bug where going from 3 motors to 4 per bus required a 100 us delay instead of 50
        // drive[i]->slewVel(pwmReceiver.channels[0]->getCh() / 100.0, pwmReceiver.channels[pwmReceiver.estop_ch]->getCh(), pwmReceiver.rcLost);
        drive[i]->slewVel(swerveKinematics.kinematics[i]->getTargetVel(), pwmReceiver.channels[pwmReceiver.estop_ch]->getCh(), pwmReceiver.rcLost);
        delayMicroseconds(can.driveCanDelay);
      } else {
        switch (bringupMode) {
          case -1:   // 0 = all DOFs simultaneously active
            steer[i]->yawTo(swerveKinematics.kinematics[i]->getTargetSteer(), eStopChannel, pwmReceiver.rcLost);
            delayMicroseconds(can.driveCanDelay);
            drive[i]->setVel(swerveKinematics.kinematics[i]->getTargetVel(), eStopChannel, pwmReceiver.rcLost);
            delayMicroseconds(can.driveCanDelay);
            break;
          case 0:   // 1 = x axis bringup
            steer[i]->yawTo(180, eStopChannel, pwmReceiver.rcLost);
            delayMicroseconds(can.driveCanDelay);
            drive[i]->setVel(traj.qd_d[0], eStopChannel, pwmReceiver.rcLost);
            delayMicroseconds(can.driveCanDelay);
            break;
          case 1: // 2 = y axis bringup
            steer[i]->yawTo(-90, eStopChannel, pwmReceiver.rcLost);  // should point forwards
            delayMicroseconds(can.driveCanDelay);
            drive[i]->setVel(traj.qd_d[1], eStopChannel, pwmReceiver.rcLost);
            delayMicroseconds(can.driveCanDelay);
            break;
          case 2: // 3 = z axis bringup
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

    plotCounter++;
    bool plot = plotCounter % 20 == 0;
    if (plot) {
      // Serial.print("Mode: ");
      // Serial.println(modes.mode);
      // Serial.print("X_d:");
      // Serial.print(traj.input[0]);
      // Serial.print(",");
      // Serial.print("Y_d:");
      // Serial.print(traj.input[1]);
      // Serial.print(",");
      // Serial.print("Z_d:");
      // Serial.println(traj.input[2]);
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
      // traj.qd_d[0] = 0;
      // traj.qd_d[1] = 0;
      // traj.qd_d[2] = 0;
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
    if (modes.debugRiding) {
      Serial.println("***********");
      Serial.println(traj.qd_d[0]);
      Serial.println(traj.qd_d[1]);
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
    steer[j]->setHoming(2);  // set homing mode to true for all axes
    steer[j]->motTo(0, pwmReceiver.channels[pwmReceiver.estop_ch]->getCh(), pwmReceiver.rcLost);
  }
  delay(750);           // Give motor time to move to zero position if it is wound up
  double fineTune = 1;  // to step in less than 1 deg increments - this is the ratio (0.2 would be in 0.2 degree increments
  double angTarget = 0;
  for (int i = 0; i < (int)(360 * abs(robotState.yRatio) * 1.5 / fineTune); i++) {
    checkRx();
    for (int j = 0; j < swerveKinematics.nWheels; j++) {
      angTarget = i * fineTune;
      can.pos = i * fineTune;
      if (steer[j]->getHoming() == 2 && digitalRead(robotState.irPin[j]) == 1) {  // calibration started with IR triggered
        steer[j]->setHoming(1);
      }
      if (steer[j]->getHoming() == 1 && digitalRead(robotState.irPin[j]) == 0) {  // Hit target fresh
        steer[j]->setYaw(robotState.irPos[j]);
        steer[j]->setMPos(can.pos);
        steer[j]->setHoming(0);
      }
      if (steer[j]->getHoming() == 0) {  // position signal should persist, but motor should stop moving after cal marker is detected
        can.pos = steer[j]->getMPos();
      }
      steer[j]->motTo(can.pos, pwmReceiver.channels[pwmReceiver.estop_ch]->getCh(), pwmReceiver.rcLost);
    }
    delayMicroseconds(800);
    doneHoming = 1;
    for (int j = 0; j < swerveKinematics.nWheels; j++) {
      doneHoming = doneHoming && steer[j]->getHoming() == 0;
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
