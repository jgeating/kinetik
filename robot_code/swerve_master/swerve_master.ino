#include "DueCANLayer.h"; // CAN library for CAN shield
#include <math.h>;        // Math functions
#include "Channel.h";     // for RC PWM inputs
#include "Kinematics.h";  // wheel level kinematics/trigonometry 
#include "Planner.h";     // robot level planning 
#include "utils.h";       // Basic utils like more powerful serial
#include "Yaw.h";         // For controlling steering actuator
#include "Pads.h";        // For interfacing with weight pads
#include "Drive.h";       // For controlling drive motors
#include <Wire.h>;        // For accessing native Arduino I2C functions
#include "Adafruit_Sensor.h"  // Downloaded library for IMU stuff
#include "Adafruit_BNO055.h"  // Downloaded library for IMU stuff
#include "utility/imumaths.h" // Downloaded library for IMU stuff

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

#define RADIUS_SWERVE_ASSEMBLY 0.25 // distance to wheel swerve axes, meters
#define YAW_GEAR_RATIO -18 // RMD-X6 planetary ratio = 8:1, pulley ratio = 72/32 = 2.25
#define DEAD_ZONE 0.1
#define pi 3.14159265358979
#define BNO055_SAMPLERATE_DELAY_MS (10)

// IMU stuff
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);  // (id, address), default 0x29 or 0x28
double alpha = 0;
imu::Vector<3> euler;

// Robot level trajectory/control
double qd_max[] = {30, 30, 30};     // max velocity: {m/s, m/s, rad/s}
double qdd_max[] = {15, 15, 30};  // max acceleration: {m/s^2, m/s^2, rad/s^2}
double dz[] = {.3, .3, .3};       // Deadzone velocity bounds: {m/s, m/s, rad/s}
double qd_d[] = {0, 0, 0};        // desired velocity, to send to planner
double qdd_d[] = {0, 0, 0};       // desired acceleration
double input[] = {0, 0, 0};       // hold inputs
Planner* planner;

// IMU variables
double x = 0; // x angle of imu vest
double y = 0; // y angle of imu vest
double z = 0; // z angle of imu vest
double maxLean = M_PI / 8;        // max lean angle, to scale to output
unsigned char getEulerCode[8] = {0xA7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int canID = 22;
int IMU_bus = 1;

// Kinematics
int nWheels = 4;
int pole_pairs = 7;   // number of pole pairs in hub motors. Assuming 7 pole pairs from research. did not actually measure. Used for converting erpm to rpm, to calculate real velocities.
int dRatio;           // used to convert target wheel rad/sec to erpm
Kinematics** kinematics = new Kinematics*[nWheels];
bool calibrated = 0;  // Whether or not the robot has been calibrated

// Loop Timing Variables
double tInner = 3000;                 // Target length of inner loop controller, microseconds
double tOuter = 50000;                // Target length of outer (second) loop controller, microseconds
unsigned long lastInner = 0;          // last time counter of inner loop
unsigned long lastOuter = 0;          // last time counter of outer loop
unsigned long now = 0;                // variable to store current time, to avoid multiple calls to micros() function
bool behind = 0;                      // If Due can't keep up with loop rate, this bool will become true
unsigned long timer[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};  // Timers used to measure computation lengths of various functions, loops, etc.

// Modes, safety, e-stop, debug
char buff[100]; // String buffer for Serial
int mode = 0;   //0 = RC mode (teleop), 1 = weight control mode
int eStop = 0;  // e-stop variable for safety
bool debugRx = 0;       // whether or not to debug receiver
bool debugTiming = 0;   // whether or not to debug timing, look at loop lengths, etc.
bool debugRiding = 0;

// CAN Stuff for drive motors
extern byte canInit(byte cPort, long lBaudRate);
extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte* cData, byte cDataLen);
extern byte canRx(byte cPort, long* lMsgID, bool* bExtendedFormat, byte* cData, byte* cDataLen);
byte cTxData0[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int idd = 1 || CAN_PACKET_SET_RPM << 8; // Drive ID
bool ext = true;
int len = 4;
uint32_t pos = 0;
// For receiving
short rxData = 0;
long lMsgID;
bool bExtendedFormat;
byte cRxData[8];
byte cDataLen;
int driveCanDelay = 100; // # of microseconds to delay sending drive CAN for
//CAN stuff for steering motors
byte cTxData1[] = {0xA3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int idy = 1 || CAN_PACKET_SET_POS << 8; // Yaw ID
unsigned char buf[8];
int32_t ang = 0;
int steerCanDelay = 100; // # of microseconds to delay sending steering CAN for

// PWM/Receiver stuff
short chs = 8;                      // number of channels to read from receiver
short chOff[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};      // Channel offsets (calibrate to find these)
Channel** channels = new Channel*[chs];
int rcTimeout = 100000;             // number of microseconds before receiver timeout is tripped - make sure it is a good bit longer than 2000 microseconds
bool rcLost = 1;                    // This is set to true if no receiver signal is received within the rcTimeout timeframe (microseconds)
int estop_ch = 6;                   // which Rx channel is used for motor enabling/SW e-stop. 0 index
int mode_ch = 7;                    // which Rx channel is used for setting mode

// Robot state stuff
int ir[] = {0, 0, 0, 0};            // status of IR sensor
//looking from bottom, (+) to offset rotates wheel CCW
double irPos[] = {254-180, 84+180, 73+180, 257-180};     // absolute position if IR sensors, for calibrating position on startup, degrees. increasing rotates clockwise looking from the top
int irPin[] = {22, 24, 26, 28};       // pins that ir sensors are hooked up to
double mRPM[] = {0, 0, 0, 0};         // Speed of drive motors (pre gear stage). In eRPM, I think...
double yRatio = YAW_GEAR_RATIO;       // Yaw pulley stage ratio, >1
int motPol[] = {1, 1, 1, 1};          // Used to switch motor direction depending on VESC configuration. Not implemented yet due to datatype issues. Just changing VESC parameters instead

Drive** drive = new Drive*[nWheels];
Yaw** yaw = new Yaw*[nWheels];
double aMaxYaw = 5000;                 // Max angular acceleration of yaw motor, in motor frame, rad/s^2. Safe starting value: 5000
double wMaxYaw = 10000;                 // Max angular velocity of yaw motor, in motor frame, rad/s. Safe starting value: 10000
int doneHoming = 0;                     // Used to determine when calibration sequence is finished. 1 = finished.

void setup()
{
  //Serial and CAN setup
  Serial.begin(115200);
  Serial.println("Initializing CAN and pinmodes...");
  if (canInit(0, CAN_BPS_1000K) == CAN_OK) Serial.print("CAN0: Initialized Successfully.\n\r");
  else Serial.print("CAN0: Initialization Failed.\n\r");
  if (canInit(1, CAN_BPS_500K) == CAN_OK) Serial.print("CAN1: Initialized Successfully.\n\r");
  else Serial.print("CAN1: Initialization Failed.\n\r");
  for (int i = 0; i < 4; i++) {
    pinMode(irPin[i], INPUT);
  }
  pinMode(13, OUTPUT); digitalWrite(13, LOW); // Set up indicator LED
  Serial.println("CAN and Pins initialized. Initializing RC interrupts...");

  // RC PWM interrupt setup
  for (int i = 0; i < chs; i++) {
    channels[i] = new Channel(CHANNEL_PIN[i], chOff[i]);
  }
  attachInterrupt(channels[0]->getPin(), calcCh1, CHANGE);
  attachInterrupt(channels[1]->getPin(), calcCh2, CHANGE);
  attachInterrupt(channels[2]->getPin(), calcCh3, CHANGE);
  attachInterrupt(channels[3]->getPin(), calcCh4, CHANGE);
  attachInterrupt(channels[4]->getPin(), calcCh5, CHANGE);
  attachInterrupt(channels[5]->getPin(), calcCh6, CHANGE);
  attachInterrupt(channels[6]->getPin(), calcCh7, CHANGE);
  attachInterrupt(channels[7]->getPin(), calcCh8, CHANGE);
  Serial.println("RC interrrupts initialized. Setting up kinematics and trajectory planning objects...");

  //  IMU
  Serial.println("Setting up IMU...");
  if (!bno.begin())
  {
     Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
     while (1);
   } else {
     Serial.println("Successfully connected to IMU");
   }
   delay(100);
   bno.setExtCrystalUse(true);
   delay(100);

  // Compute alpha
   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
   alpha = euler.x() * pi / 180.0;
   Serial.println(alpha);
   delay(50);

  // Kinematics and path planning setup
  dRatio = pole_pairs * 60 / (2 * M_PI) / (.083 / 2); // used to convert m/s to rpm
  for (int i = 0; i < nWheels; i++) {
    irPos[i] = irPos[i];    // Correcting for polar coordinate frame
    yaw[i] = new Yaw(wMaxYaw, aMaxYaw, yRatio, tInner, len, i);
    drive[i] = new Drive(qd_max[0], qdd_max[0], dRatio, tInner, len, i);
    kinematics[i] = new Kinematics(RADIUS_SWERVE_ASSEMBLY, DEAD_ZONE, i);
  }
  planner = new Planner(tInner, qd_max[0], qd_max[1], qd_max[2], qdd_max[0], qdd_max[1], qdd_max[2], dz[0], dz[1], dz[2], mode, maxLean);
  Serial.println("Startup Complete.");
}

void loop()
{
  now = micros();
  timer[0] = now;
  if (now - lastInner > tInner) {
    if (behind) { // This means we can't keep up with the desired loop rate. Trip LED to indicate so
      digitalWrite(13, HIGH);
      lastInner = now;
    } else {
      digitalWrite(13, LOW);
      lastInner = lastInner + tInner;
      behind = true;
    }
     
    //***************BEGIN FAST LOOP*******************
    if (mode == 0) { // tele-op mode
      for (int k = 0; k < 3; k++) {
        qd_d[k] = constrain(channels[k + 1]->getCh(), -500, 500);
        qd_d[k] = qd_d[k] * qd_max[k] / 500.0;
        if ( k < 2 ) {
          qd_d[k] = qd_d[k] * float(constrain(channels[0]->getCh(), -500, 500)/1000.0 + 0.5);    // Scale up to max velocity using left stick vertical (throttle, no spring center)
        }
      }
      
      // compute alpha
      timer[1] = micros();
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      alpha = euler.x() * pi / 180.0;
      timer[2] = micros();
//      Serial.println(alpha);
//      delay(50);

//      planner->plan(qd_d[0], qd_d[1], qd_d[2]);
      planner->plan_world(qd_d[0], qd_d[1], qd_d[2], alpha);

      //sprintf(buff, "Inputs: x: %.2f m/s, y: %.2f m/s, z: %.2f m/s", qd_d[0], qd_d[1], qd_d[2]);
      //Serial.println(buff);

    } else if (mode == 1 || mode == 2 || mode == 3) { // IMU modes. 1 = zero, 2 = velocity, 3 = acceleration
      canTx( IMU_bus, canID, false, getEulerCode,   8);
      delayMicroseconds(100);
      bool buffed = 1;
      while (buffed) {
        if (canRx(IMU_bus, &lMsgID, &bExtendedFormat, &cRxData[0], &cDataLen) == CAN_OK) {
          if (lMsgID == canID) {
            int16_t temp = (int16_t)(cRxData[0] + (cRxData[1] << 8));
            z =  (double)((int16_t)(cRxData[2] + (cRxData[3] << 8))) / 100.0;
            x = -(double)((int16_t)(cRxData[4] + (cRxData[5] << 8))) / 100.0;
            y =  (double)((int16_t)(cRxData[6] + (cRxData[7] << 8))) / 100.0;
            input[0] = x;
            input[1] = y;
            input[2] = z;
            // sprintf(buff, "Inputs: x: %.2f°, y: %.2f°, z: %.2f°", x, y, z);
            // Serial.println(buff);

            for (int i = 0; i < 3; i++) {
              input[i] = input[i] * M_PI / 180; // Convert inputs to radians
            }
            planner->plan(input[0], input[1], input[2]);
          }
        } else {
          buffed = 0;
        }
      }
    }

    // Perform planning
    qd_d[0] = planner->getTargetVX();
    qd_d[1] = planner->getTargetVY();
    qd_d[2] = planner->getTargetVZ();

    // sprintf(buff, "Outputs: x: %.3f m/s, y: %.3f m/s, z: %.4f rad/s", qd_d[0], qd_d[1], qd_d[2]);
    // Serial.println(buff);
    // delay(50);

    //    Serial.println("Wheel outputs: ");
    for (int i = 0; i < nWheels; i++) {
      kinematics[i]->calc(qd_d[0], qd_d[1], qd_d[2]*sign(yRatio));
      //      Serial.print(kinematics[i]->getTargetYaw());
      //      Serial.print(", ");
      //      Serial.println(kinematics[i]->getTargetVel());
    }
    //delay(30);

    for (int i = 0; i < nWheels; i++) {
      yaw[i]->yawTo(kinematics[i]->getTargetYaw(), channels[estop_ch]->getCh(), rcLost);
      delayMicroseconds(steerCanDelay); // Nasty bug where going from 3 motors to 4 per bus required a 100 us delay instead of 50
      drive[i]->setVel(kinematics[i]->getTargetVel(), channels[estop_ch]->getCh(), rcLost);
      delayMicroseconds(driveCanDelay);
    }
    timer[4] = micros();
  } else {
    behind = false;
  }

  if (now - lastOuter > tOuter) {
    lastOuter = lastOuter + tOuter;
    //***********************BEGIN SLOW LOOP*******************************
    checkRx();                          // check if receiver signal has been lost

    // Check channels, modes
    if (channels[5]->getCh() > 400) {
      calMotor();  // Calibrate motors
    }
    if (channels[estop_ch]->getCh() < -400 && channels[estop_ch]->getCh() < 100) { // calibrate force pads, only if steering motoors are off
      qd_d[0] = 0;
      qd_d[1] = 0;
      qd_d[2] = 0;
    }
    if (channels[mode_ch]->getCh() < -300) {  // default mode is tele-op, blue stick top position
      mode = 0;
      planner->setMode(0);
      eStop = 0; //Also disable e-stop if tripped
    }
    if (channels[mode_ch]->getCh() > -300 && channels[mode_ch]->getCh() < 300) { // IMU zeroing mode
      mode = 1;
      planner->setMode(1);
    }
    if (channels[mode_ch]->getCh() > 300) { // riding mode, blue stick down position
      mode = 2;
      planner->setMode(2);
    }
    timer[5] = micros();

    // Debug related
    if (debugRiding) {
      Serial.println("***********");
      Serial.println(qd_d[0]);
      Serial.println(qd_d[1]);
      delay(20);
    }
    if (debugRx) {
      // printChannels(); // printChannels() has been deleted
      delay(20);
    }
    if (debugTiming) {
      Serial.println();
      Serial.println();
      char buff[40];
      sprintf(buff, "prior to imu poll:                %i usec", timer[1] - timer[0]);
      Serial.println(buff);
      sprintf(buff, "After IMU poll:                   %i usec", timer[2] - timer[0]);
      Serial.println(buff);
      sprintf(buff, "unused:                          %i usec", timer[3] - timer[0]);
      Serial.println(buff);
      sprintf(buff, "After sending CAN commands:       %i usec", timer[4] - timer[0]);
      Serial.println(buff);
      sprintf(buff, "After outer loop:                 %i usec", timer[5] - timer[0]);
      Serial.println(buff);
      sprintf(buff, "After Serial output (debug only): %i usec", micros() - timer[0]);
      Serial.println(buff);
      delay(20);
    }
  }
}

// *********************************************** HELPER FUNCTIONS **************************************************************

// ************** CALIBRATION **********************
void calMotor() {
  for (int j = 0; j < nWheels; j++) {
    yaw[j]->setHoming(2);               // set homing mode to true for all axes
    yaw[j]->motTo(0, channels[estop_ch]->getCh(), rcLost);
  }
  delay(750); // Give motor time to move to zero position if it is wound up
  double fineTune = 1;    // to step in less than 1 deg increments - this is the ratio (0.2 would be in 0.2 degree increments
  double angTarget = 0;
  for (int i = 0; i < (int)(360 * abs(yRatio) * 1.5 / fineTune); i++) {
    checkRx();
    for (int j = 0; j < nWheels; j++) {
      angTarget = i * fineTune;
      pos = i * fineTune;
      if (yaw[j]->getHoming() == 2 && digitalRead(irPin[j]) == 1) { // calibration started with IR triggered
        yaw[j]->setHoming(1);
      }
      if (yaw[j]->getHoming() == 1 && digitalRead(irPin[j]) == 0) { // Hit target fresh
        yaw[j]->setYaw(irPos[j]);
        yaw[j]->setMPos(pos);
        yaw[j]->setHoming(0);
      }
      if (yaw[j]->getHoming() == 0) {   // position signal should persist, but motor should stop moving after cal marker is detected
        pos = yaw[j]->getMPos();
      }
      yaw[j]->motTo(pos, channels[estop_ch]->getCh(), rcLost);
    }
    delayMicroseconds(800);
    doneHoming = 1;
    for (int j = 0; j < nWheels; j++) {
      doneHoming = doneHoming && yaw[j]->getHoming() == 0;
    }
    if (doneHoming == 1) {
      calibrated = 1; // set calibration flag to true
    }
    if (doneHoming == 1) break; // break from loop once all targets have been found
  }
}

double mapDouble(double x, double min_in, double max_in, double min_out, double max_out) {
  double ret = (x - min_in) / (max_in - min_in);
  ret = ret * (max_out - min_out) + min_out;
  return ret;
}

// This function detects if a receiver signal has been received recently. Used for safety, etc.
void checkRx() {
  if (micros() - channels[estop_ch]->getLastInterruptTime() > rcTimeout) {
    rcLost = 1;
  } else {
    rcLost = 0;
  }
}

// ************************* CAN RECEIVER
void rxMsg() {
  if (canRx(0, &lMsgID, &bExtendedFormat, &cRxData[0], &cDataLen) == CAN_OK)
  {
    rxData = 0;
    for (int i = 0; i < len; i++) {
      rxData = rxData << 8;
      rxData |=  cRxData[i];
    }
  }
}

// ***********************2.4 GHz RECEIVER  FUNCTIONS
void calcCh1() {
  channels[0]->calc();
}
void calcCh2() {
  channels[1]->calc();
}
void calcCh3() {
  channels[2]->calc();
}
void calcCh4() {
  channels[3]->calc();
}
void calcCh5() {
  channels[4]->calc();
}
void calcCh6() {
  channels[5]->calc();
}
void calcCh7() {
  channels[6]->calc();
}
void calcCh8() {
  channels[7]->calc();
}
