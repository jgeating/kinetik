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

// IMU stuff
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29);      // (id, address), default 0x29 or 0x28
Adafruit_BNO055 bnoVest = Adafruit_BNO055(-1, 0x28);  // (id, address), default 0x29 or 0x28
double vestAngleCenter = 0;

double alpha = 0;
imu::Vector<3> euler;

SwerveTrajectory swerveTrajectory;
Planner *planner;
SwerveImu swerveImu;
SwerveKinematics swerveKinematics;
LoopTiming loopTiming;
Profiles profiles;

// Modes, safety, e-stop, debug
Modes modes;

// CAN Stuff for drive motors
extern byte canInit(byte cPort, long lBaudRate);
extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte *cData, byte cDataLen);
extern byte canRx(byte cPort, long *lMsgID, bool *bExtendedFormat, byte *cData, byte *cDataLen);
SwerveCAN can;

PWMReceiver pwmReceiver;  // PWM/Receiver stuff
RobotState robotState;
Drive **drive = new Drive *[swerveKinematics.nWheels];
Yaw **yaw = new Yaw *[swerveKinematics.nWheels];
double aMaxYaw = 5000;   // Max angular acceleration of yaw motor, in motor frame, rad/s^2. Safe starting value: 5000
double wMaxYaw = 10000;  // Max angular velocity of yaw motor, in motor frame, rad/s. Safe starting value: 10000
int doneHoming = 0;      // Used to determine when calibration sequence is finished. 1 = finished.

int plotCounter = 0;

unsigned long prevTelemetryReportTime = 0;

Watchdog watchdog;

class LowPassFilter {
  private:
    int numReadings;     // number of readings to be averaged
    int* readings;       // array to store the readings
    int index;           // index of the current reading
    int total;           // running total of the readings
    int average;         // calculated average value
  public:
    LowPassFilter(int n) {
      numReadings = n;
      readings = new int[numReadings];
      for (int i = 0; i < numReadings; i++) {
        readings[i] = 0;
      }
      index = 0;
      total = 0;
      average = 0;
    }
    ~LowPassFilter() {
      delete[] readings;
    }
    int filter(int value) {
      // subtract the last reading:
      total -= readings[index];
      // add the new reading to the total:
      total += value;
      // store the new reading in the array:
      readings[index] = value;
      // advance to the next position in the array:
      index++;
      // if we've reached the end of the array, wrap around to the beginning:
      if (index >= numReadings) {
        index = 0;
      }
      // calculate the average value:
      average = total / numReadings;
      // return the filtered value:
      return average;
    }
};

LowPassFilter filter(10);  // create a low-pass filter with 10 readings


void setup() {
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

  //  IMU
  // setupImu("robot IMU", bno);
  // setupImu("robot Vest", bnoVest);

  // bno.setExtCrystalUse(true);
  // delay(100);

  // Compute alpha
  // imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  alpha = euler.x() * pi / 180.0;
  Serial.println(alpha);
  delay(50);

  // Kinematics and path planning setup
  swerveKinematics.dRatio = swerveKinematics.pole_pairs * 60 / (2 * M_PI) / (.083 / 2);  // used to convert m/s to rpm
  for (int i = 0; i < swerveKinematics.nWheels; i++) {
    robotState.irPos[i] = robotState.irPos[i];  // Correcting for polar coordinate frame
    yaw[i] = new Yaw(wMaxYaw, aMaxYaw, robotState.yRatio, loopTiming.tInner, can.len, i);
    drive[i] = new Drive(swerveTrajectory.qd_max[0], swerveTrajectory.qdd_max[0], swerveKinematics.dRatio, loopTiming.tInner, can.len, i);
    swerveKinematics.kinematics[i] = new Kinematics(RADIUS_SWERVE_ASSEMBLY, DEAD_ZONE, i);
  }
  planner = new Planner(loopTiming.tInner, swerveTrajectory.qd_max[0], swerveTrajectory.qd_max[1], swerveTrajectory.qd_max[2], swerveTrajectory.qdd_max[0], swerveTrajectory.qdd_max[1], swerveTrajectory.qdd_max[2], swerveTrajectory.dz[0], swerveTrajectory.dz[1], swerveTrajectory.dz[2], modes.mode, swerveImu.maxLean);
  Serial.println("Startup Complete.");
}

void loop() {
  startProfile(profiles.robotLoop);
  // printWatchdogError(watchdog);
  // telemetry();
  startProfile(profiles.centerVestAngle);
  centerVestAngle();
  endProfile(profiles.centerVestAngle);

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

    //***************BEGIN FAST LOOP*******************
    if (modes.mode == 0) {
      startProfile(profiles.mode0);

      // tele-op mode
      for (int k = 0; k < 3; k++) {
        swerveTrajectory.qd_d[k] = constrain(pwmReceiver.channels[k + 1]->getCh(), -500, 500);
        swerveTrajectory.qd_d[k] = swerveTrajectory.qd_d[k] * swerveTrajectory.qd_max[k] / 500.0;
        if (k < 2) {
          swerveTrajectory.qd_d[k] = swerveTrajectory.qd_d[k] * float(constrain(pwmReceiver.channels[0]->getCh(), -500, 500) / 1000.0 + 0.5);  // Scale up to max velocity using left stick vertical (throttle, no spring center)
        }
      }
      
      if (plotCounter % 20 == 0 && false) {
        Serial.print("pwm 0:");
        Serial.print(pwmReceiver.channels[0]->getCh());
        Serial.print("pwm 1:");
        Serial.print(pwmReceiver.channels[1]->getCh());
        Serial.print("pwm 2:");
        Serial.print(pwmReceiver.channels[2]->getCh());
        Serial.print("pwm 3:");
        Serial.print(pwmReceiver.channels[3]->getCh());
        Serial.print("\n");
      }

      // compute alpha
      loopTiming.timer[1] = micros();
      // euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      // alpha = euler.x() * pi / 180.0;
      loopTiming.timer[2] = micros();
      //      Serial.println(alpha);
      //      delay(50);

      //      planner->plan(qd_d[0], qd_d[1], qd_d[2]);
      // planner->plan_world(swerveTrajectory.qd_d[0], swerveTrajectory.qd_d[1], swerveTrajectory.qd_d[2], alpha);
      planner->plan_world(swerveTrajectory.qd_d[0], swerveTrajectory.qd_d[1], -swerveTrajectory.qd_d[2], 0);

      // sprintf(buff, "Inputs: x: %.2f m/s, y: %.2f m/s, z: %.2f m/s", qd_d[0], qd_d[1], qd_d[2]);
      // Serial.println(buff);
      endProfile(profiles.mode0);
    } else if (modes.mode == 1 || modes.mode == 2 || modes.mode == 3) {
      startProfile(profiles.modeOther);

      // IMU modes. 1 = zero, 2 = velocity, 3 = acceleration
      // canTx(swerveImu.IMU_bus, swerveImu.canID, false, swerveImu.getEulerCode, 8);
      delayMicroseconds(100);
      bool buffed = 1;
      while (buffed) {
        if (canRx(swerveImu.IMU_bus, &can.lMsgID, &can.bExtendedFormat, &can.cRxData[0], &can.cDataLen) == CAN_OK) {
          if (can.lMsgID == swerveImu.canID) {
            int16_t temp = (int16_t)(can.cRxData[0] + (can.cRxData[1] << 8));
            swerveImu.z = (double)((int16_t)(can.cRxData[2] + (can.cRxData[3] << 8))) / 100.0;
            swerveImu.x = -(double)((int16_t)(can.cRxData[4] + (can.cRxData[5] << 8))) / 100.0;
            swerveImu.y = (double)((int16_t)(can.cRxData[6] + (can.cRxData[7] << 8))) / 100.0;
            swerveTrajectory.input[0] = swerveImu.x;
            swerveTrajectory.input[1] = swerveImu.y;
            swerveTrajectory.input[2] = -swerveImu.z;
            // sprintf(buff, "Inputs: x: %.2f°, y: %.2f°, z: %.2f°", x, y, z);
            // Serial.println(buff);

            for (int i = 0; i < 3; i++) {
              swerveTrajectory.input[i] = swerveTrajectory.input[i] * M_PI / 180;  // Convert inputs to radians
            }
            planner->plan(swerveTrajectory.input[0], swerveTrajectory.input[1], swerveTrajectory.input[2]);
          }
        } else {
          buffed = 0;
        }
      }
      endProfile(profiles.modeOther);
    }

    startProfile(profiles.kinematics);

    // Perform planning
    swerveTrajectory.qd_d[0] = planner->getTargetVX();
    swerveTrajectory.qd_d[1] = planner->getTargetVY();
    swerveTrajectory.qd_d[2] = planner->getTargetVZ();

    // sprintf(buff, "Outputs: x: %.3f m/s, y: %.3f m/s, z: %.4f rad/s", qd_d[0], qd_d[1], qd_d[2]);
    // Serial.println(buff);
    // delay(50);

    //    Serial.println("Wheel outputs: ");
    for (int i = 0; i < swerveKinematics.nWheels; i++) {
      swerveKinematics.kinematics[i]->calc(swerveTrajectory.qd_d[0], swerveTrajectory.qd_d[1], swerveTrajectory.qd_d[2] * sign(robotState.yRatio));
      //      Serial.print(kinematics[i]->getTargetYaw());
      //      Serial.print(", ");
      //      Serial.println(kinematics[i]->getTargetVel());
    }
    // delay(30);
    endProfile(profiles.kinematics);

    startProfile(profiles.getImuZForVest);
    // double relativeAngle = (vestAngleCenter - getImuZ(bnoVest));
    endProfile(profiles.getImuZForVest);

    startProfile(profiles.updateMotorSpeeds);
    int eStopChannel = pwmReceiver.channels[pwmReceiver.estop_ch]->getCh();
    bool teleop = pwmReceiver.channels[pwmReceiver.mode_ch]->getCh() < -300;

    plotCounter++;
    bool plot = plotCounter % 20 == 0;
    for (int i = 0; i < swerveKinematics.nWheels; i++) {
      if (teleop) {
        yaw[i]->yawTo(swerveKinematics.kinematics[i]->getTargetYaw(), pwmReceiver.channels[pwmReceiver.estop_ch]->getCh(), pwmReceiver.rcLost);
        delayMicroseconds(can.steerCanDelay);  // Nasty bug where going from 3 motors to 4 per bus required a 100 us delay instead of 50
        // drive[i]->slewVel(pwmReceiver.channels[0]->getCh() / 100.0, pwmReceiver.channels[pwmReceiver.estop_ch]->getCh(), pwmReceiver.rcLost);
        drive[i]->slewVel(swerveKinematics.kinematics[i]->getTargetVel(), pwmReceiver.channels[pwmReceiver.estop_ch]->getCh(), pwmReceiver.rcLost);
        if (plot) {
          // drive[i]->slewVel(swerveKinematics.kinematics[i]->getTargetVel(), pwmReceiver.channels[pwmReceiver.estop_ch]->getCh(), pwmReceiver.rcLost);
          // Serial.print("Target Vel ");
          // Serial.print(i);
          // Serial.print(":");
          // Serial.print(swerveKinematics.kinematics[i]->getTargetVel());
        }
        delayMicroseconds(can.driveCanDelay);
      } else {
        yaw[i]->yawTo(90, eStopChannel, pwmReceiver.rcLost);
        // double velocity = constrain(relativeAngle * 0.1, -3.0, 3.0);
        double velocity = 0;
        drive[i]->setVel(velocity, eStopChannel, pwmReceiver.rcLost);
        // drive[i]->setVel()
      }
    }
    if (plot) {
      // Serial.print("\n");
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
      swerveTrajectory.qd_d[0] = 0;
      swerveTrajectory.qd_d[1] = 0;
      swerveTrajectory.qd_d[2] = 0;
    }
    if (pwmReceiver.channels[pwmReceiver.mode_ch]->getCh() < -300) {  // default mode is tele-op, blue stick top position
      modes.mode = 0;
      planner->setMode(0);
      modes.eStop = 0;  // Also disable e-stop if tripped
    }
    if (pwmReceiver.channels[pwmReceiver.mode_ch]->getCh() > -300 && pwmReceiver.channels[pwmReceiver.mode_ch]->getCh() < 300) {  // IMU zeroing mode
      modes.mode = 1;
      planner->setMode(1);
    }
    if (pwmReceiver.channels[pwmReceiver.mode_ch]->getCh() > 300) {  // riding mode, blue stick down position
      modes.mode = 2;
      planner->setMode(2);
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
      break;  // break from loop once all targets have been found
  }
}

double mapDouble(double x, double min_in, double max_in, double min_out, double max_out) {
  double ret = (x - min_in) / (max_in - min_in);
  ret = ret * (max_out - min_out) + min_out;
  return ret;
}

void centerVestAngle() {
  int reset = pwmReceiver.channels[6]->getCh();
  if (reset < 400) {
    vestAngleCenter = getImuZ(bnoVest);
  }
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

double getImuZ(Adafruit_BNO055 &imu) {
  imu::Vector<3> euler = imu.getVector(Adafruit_BNO055::VECTOR_EULER);
  return euler.z();
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
