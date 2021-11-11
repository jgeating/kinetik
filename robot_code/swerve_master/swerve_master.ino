#include "DueCANLayer.h"; // CAN library for CAN shield
#include <math.h>;        // Math functions
#include "Channel.h";     // for RC PWM inputs
#include "Kinematics.h";  // wheel level kinematics/trigonometry 
#include "Planner.h";     // robot level planning 
#include "utils.h";       // Basic utils like more powerful serial
#include "Yaw.h";         // For controlling steering actuator
#include "Pads.h";        // For interfacing with weight pads
#include "Drive.h";       // For controlling drive motors
#include "Wire.h";        // For accessing native Arduino I2C functions

const int CHANNEL_PIN[] = {
  24,  // left stick vertical, forward = (+)
  25,  // right stick horizontal, R = (+)
  26,  // right stick vertical, forward = (+)
  27,  // left stick horizontal, R = (+)
  28,  // left knob, CW = (+)
  29,  // right knob, CW = (+)
  30,  // left switch, backwards = (-)
  31   // right switch, backwards = (-)
};

#define RADIUS_SWERVE_ASSEMBLY 0.25 // distance to wheel swerve axes, meters
#define YAW_GEAR_RATIO -18 // RMD-X6 planetary ratio = 8:1, pulley ratio = 72/32 = 2.25
#define DEAD_ZONE 0.05
#define pi 3.14159265358979

extern byte canInit(byte cPort, long lBaudRate);
extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte* cData, byte cDataLen);
extern byte canRx(byte cPort, long* lMsgID, bool* bExtendedFormat, byte* cData, byte* cDataLen);

// Robot level trajectory/control
double qdd_max[] {10, 10, 20};   // max acceleration: {m/s^2, m/s^2, rad/s^2}
double qd_max[] = {5, 5, 10};  // max velocity: {m/s, m/s, rad/s}
double dz[] = {qd_max[0]*.1, qd_max[1]*.1, qd_max[2]*.1};   // Deadzone velocity bounds: {m/s, m/s, rad/s}
double qd_d[] = {0, 0, 0};      // desired velocity, to send to planner
double qdd_d[] = {0, 0, 0};     // desired acceleration, obsolete
Planner* planner;

// IMU variables
double x = 0; // x angle of imu vest
double y = 0; // y angle of imu vest
double z = 0; // z angle of imu vest
unsigned char getEulerCode[8] = {0xA7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int canID = 22;

// Kinematics
int nWheels = 4;
int pole_pairs = 7;   // number of pole pairs in hub motors. Assuming 7 pole pairs from research. did not actually measure. Used for converting erpm to rpm, to calculate real velocities.
int dRatio;           // used to convert target wheel rad/sec to erpm
Kinematics** kinematics = new Kinematics*[nWheels];
bool calibrated = 0;  // Whether or not the robot has been calibrated

// Loop Timing Variables
double tInner = 2000;                 // Target length of inner loop controller, microseconds
double tOuter = 50000;                // Target length of outer (second) loop controller, microseconds
unsigned long lastInner = 0;          // last time counter of inner loop
unsigned long lastOuter = 0;          // last time counter of outer loop
unsigned long now = 0;                // variable to store current time, to avoid multiple calls to micros() function
bool behind = 0;                      // If Due can't keep up with loop rate, this bool will become true
unsigned long timer[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};  // Timers used to measure computation lengths of various functions, loops, etc.

// Modes, safety, e-stop, debug
int mode = 0;   //0 = RC mode (teleop), 1 = weight control mode
int eStop = 0;  // e-stop variable for safety
bool debugRx = 0;       // whether or not to debug receiver
bool debugTiming = 0;   // whether or not to debug timing, look at loop lengths, etc.
bool debugRiding = 0;

// CAN Stuff for drive motors
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

// Receiver stuff
short chs = 8;                      // number of channels to read from receiver
short chOff[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};      // Channel offsets (calibrate to find these)
Channel** channels = new Channel*[chs];
int rcTimeout = 100000;             // number of microseconds before receiver timeout is tripped - make sure it is a good bit longer than 2000 microseconds
bool rcLost = 1;                    // This is set to true if no receiver signal is received within the rcTimeout timeframe (microseconds)
int estop_ch = 6;                   // which Rx channel is used for motor enabling/SW e-stop. 0 index
int mode_ch = 7;                    // which Rx channel is used for setting mode

//IMU and PWM comms stuff (uses channels)
double imu_pwm_freq = 488.28; // frequency of PWM, used for calculating/converting timing params to control signals, and centering signal. 7/3/21: not currently used
double deadband = 0.05; // unused duty cycle of PWM on either extreme (ensures interrupts can be used to catch pin changes). 7/3/21: not currently used
short imu_chs = 2;
short imu_min_pulse[] = {110, 102};    // emperically determined max duty cycle, us
short imu_max_pulse[] = {2002, 1990}; // emperically determined min duty cycle, us
short imu_chOff[] = {0, 0};           // IMU offsets - to set zeroed tilt position
short imu_zero[] = {imu_min_pulse[0], imu_min_pulse[1]};
short imu_pulse_range[] = {(imu_max_pulse[0] - imu_min_pulse[0]), (imu_max_pulse[1] - imu_min_pulse[1])};
const int imu_ch_pins[] = {2, 3}; //imu channel digital pins
Channel** imu_channels = new Channel*[imu_chs];
double orientation[] = {0, 0};  //stores orientation in degrees
double pulse2deg[] = {360.0 / (double)imu_pulse_range[0], 360.0 / (double)imu_pulse_range[1]};

// Robot state stuff
int ir[] = {0, 0, 0, 0};            // status of IR sensor
//looking from bottom, (+) to offset rotates wheel CCW
double irPos[] = {254, 84, 73, 257};     // absolute position if IR sensors, for calibrating position on startup, degrees. increasing rotates clockwise looking from the top
// 8/8/2021 values were 158, 347, 342, 171
int irPin[] = {35, 34, 33, 32};       // pins that ir sensors are hooked up to
double mRPM[] = {0, 0, 0, 0};         // Speed of drive motors (pre gear stage). In eRPM, I think...
double yRatio = YAW_GEAR_RATIO;       // Yaw pulley stage ratio, >1
int motPol[] = {1, 1, 1, 1};          // Used to switch motor direction depending on VESC configuration. Not implemented yet due to datatype issues. Just changing VESC parameters instead

Drive** drive = new Drive*[nWheels];
Yaw** yaw = new Yaw*[nWheels];
double aMaxYaw = 5000;                 // Max angular acceleration of yaw motor, in motor frame, rad/s^2. Safe starting value: 5000
double wMaxYaw = 10000;                 // Max angular velocity of yaw motor, in motor frame, rad/s. Safe starting value: 10000
double xYawCoeff[] = {0, 0, 0, 0};      
double yYawCoeff[] = {0, 0, 0, 0};      
int doneHoming = 0;                     // Used to determine when calibration sequence is finished. 1 = finished.

// Force Sensor Variables
// Used pins: Analog                0,  1,  2,  3,             4,  5,  6,  7
// In the above order, Right Side: FL, FR, BR, BL, Left Side: BR, BL, FR, FL
// Order for programming:       R: FL, FR, BL, BR,         L: FL, FR, BL, BR
int forcepins[] =                 {A0, A1, A3, A2,            A7, A6, A5, A4}; // Select the input pins for the load cells
int forcezeros[] = {1995, 1852, 2229, 2348, 2152, 2081, 2007, 2305}; // Zero force values with pads on top of sensors - in forcepins[] order
double lccapacity = 50;                         // Load cell capacity, kg
int anRes = 12;                                 // analogRead resolution, bits
double lcgain = lccapacity / (pow(2, anRes) / 2); // Scaling constant to convert from analog value to kilogram-force on load cell
double weightthresh = 30;                       //min weight to not enable estop
//Pads* pads = new Pads(forcepins, forcezeros, xgain, ygain, zgain, xyMax, wMax, weightthresh);

// Thumbstick setup
int thumbpins[] =  { A0,   A1,   A2,   A3};
int thumbzeros[] = { 2112, 2066, 2060, 2113};
//                   LV,   LH,   RH,   RV
// Positive signal:  DN,   L ,   L ,   DN

void setup()
{
  // Setup hardware
  Serial.begin(115200);
  Serial.println("Starting up...");
  analogReadResolution(anRes);

  Serial.println("Initializing CAN and pinmodes...");
  if (canInit(0, CAN_BPS_1000K) == CAN_OK)
    Serial.print("CAN0: Initialized Successfully.\n\r");
  else
    Serial.print("CAN0: Initialization Failed.\n\r");
  if (canInit(1, CAN_BPS_500K) == CAN_OK)
    Serial.print("CAN1: Initialized Successfully.\n\r");
  else
    Serial.print("CAN1: Initialization Failed.\n\r");
  pinMode(irPin[0], INPUT);
  pinMode(irPin[1], INPUT);
  pinMode(irPin[2], INPUT);
  pinMode(irPin[3], INPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  //  initImu();
  //  displayImuSensorDetails();
  //  displayImuSensorStatus();
  //  setImuExtCrystalUse(true);
  Serial.println("CAN and Pins initialized. Initializing RC interrupts...");

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

  Serial.println("RC interrrupts initialized. Calculating Kinematics and constants...");

  // Kinematics and constants calculations
  Serial.println("Initializing yaw, drive and kinematics objects.");
  dRatio = pole_pairs * 60 / (2*M_PI) / (.083/2); // used to convert m/s to rpm
  for (int i = 0; i < nWheels; i++) {
    irPos[i] = irPos[i];    // Correcting for polar coordinate frame
    yaw[i] = new Yaw(wMaxYaw, aMaxYaw, yRatio, tInner, len, i);
    drive[i] = new Drive(qd_max[0], qdd_max[0], dRatio, tInner, len, i);
    kinematics[i] = new Kinematics(RADIUS_SWERVE_ASSEMBLY, DEAD_ZONE, i);
  }
  planner = new Planner(tInner, qd_max[0], qd_max[1], qd_max[2], qdd_max[0], qdd_max[1], qdd_max[2], dz[0], dz[1], dz[2]);
  Serial.println("Startup Complete.");
}

void printChannels() {
  Serial.println("");
  Serial.println("---------");
  for (int i = 0; i < chs; i++)
  {
    Serial.print("Pin: ");
    Serial.print(CHANNEL_PIN[i]);
    Serial.print(", ");
    Serial.print(i);
    Serial.print(":");
    Serial.println(channels[i]->getCh());
  }
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
    timer[1] = micros(); 

    //***************BEGIN INNER LOOP**************************
    //imu::Vector<3> angle = getAngle();

    // in tele-op mode
    if (mode == 0) {
      //Serial.println("Desired vels: ");
      for (int k = 0; k < 3; k++) {
        qd_d[k] = constrain(channels[k + 1]->getCh(), -500, 500);
        qd_d[k] = qd_d[k] * qd_max[k] / 500.0; 
        //Serial.println(qd_d[k]);
      }
      timer[3] = micros();

      // Thumbstick mode
    } else if (mode == 1) {
      qd_d[0] = constrain((double)analogRead(thumbpins[2]) - thumbzeros[2], -2000, 2000) / (-2000.0) * qd_max[0];
      qd_d[1] = constrain((double)analogRead(thumbpins[3]) - thumbzeros[3], -2000, 2000) / (-2000.0) * qd_max[1];
      qd_d[2] = constrain((double)analogRead(thumbpins[1]) - thumbzeros[1], -2000, 2000) / (-2000.0) * qd_max[2];
      // in riding mode
    } else if (mode == 2) {
      canTx(  0,  canID, false, getEulerCode,   8); 
      //delay(500);
      //delayMicroseconds(5000);
      //    Bus, ID,   ext,          dat, len 
      // *Currently no delay between Tx and Rx. Might be source of error to check in the future
      bool buffed = 1;
      while (buffed){
      if(canRx(0, &lMsgID, &bExtendedFormat, &cRxData[0], &cDataLen) == CAN_OK){
        if(lMsgID == canID){
          buffed = 0;
          int16_t temp = (uint16_t)(cRxData[0] + (cRxData[1] << 8));
          x = (uint16_t)(cRxData[2] + (cRxData[3] << 8));   
          y = (uint16_t)(cRxData[4] + (cRxData[5] << 8));    
          z = (uint16_t)(cRxData[6] + (cRxData[7] << 8));   
          orientation[0] = y;
          orientation[1] = z;
          
          /*
          Serial.print("temp: ");
          Serial.print(temp);
          Serial.print(", x: ");
          Serial.print(x);
          Serial.print(", y: ");
          Serial.print(y);
          Serial.print(", z: ");
          Serial.println(z);
          */

          Serial.print("CAN0: Rx - MsgID:");
          Serial.print(lMsgID, HEX);
          Serial.print(" Ext:");
          Serial.print(bExtendedFormat);
          Serial.print(" Len:");
          Serial.print(cDataLen);
          Serial.print(" Data:");
  
          for(byte cIndex = 0; cIndex < cDataLen; cIndex++)
          {
            Serial.print(cRxData[cIndex], HEX);
            Serial.print(" ");
          }// end for
  
          Serial.print("\n\r");      
        }
      }
      }
      
      while abs(orientation[0] > 180) {
        if (orientation[0] > 180) {
          orientation[0] = orientation[0] - 360;
        }
        if (orientation[0] < -180) {
          orientation[0] = orientation[0] + 360;
        }
      }
      while abs(orientation[1] > 180) {
        if (orientation[1] > 180) {
          orientation[1] = orientation[1] - 360;
        }
        if (orientation[1] < -180) {
          orientation[1] = orientation[1] + 360;
        }
      }
      //qdd_d[0] = orientation[0] / 45; // for accel control
      //qdd_d[1] = orientation[1] / 45; // for accel control
      qd_d[0] = orientation[0] / 45;
      qd_d[1] = orientation[1] / 45;
      qd_d[2] = constrain(channels[2 + 1]->getCh(), -500, 500);
      qd_d[2] = qd_d[2] * qd_max[2] / 500.0; 
      for (int i = 0; i < 2; i++) {
        //qd_d[i] = qd_d[i] + (qdd_d[i] * tInner / 1000000 * qd_max[0]);  // for acceleration control
        qd_d[i] = constrain(qd_d[i], -qd_max[i], qd_max[i]);
      }
      /*
      Serial.print("Vel x: ");
      Serial.print(qd_d[0]);
      Serial.print(", Vel y: ");
      Serial.print(qd_d[1]);
      Serial.print(", vel z: ");
      Serial.println(qd_d[2]);
      */
      delay(20);
    }

    // Perform planning
    planner->calc(qd_d[0], qd_d[1], qd_d[2]);
    qd_d[0] = planner->getTargetVX();
    qd_d[1] = planner->getTargetVY();
    qd_d[2] = planner->getTargetVZ();
    
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
      canRx(0, &lMsgID, &bExtendedFormat, &cRxData[0], &cDataLen);
      drive[i]->slewVel(kinematics[i]->getTargetVel(), channels[estop_ch]->getCh(), rcLost);
      delayMicroseconds(driveCanDelay);
    }
    timer[4] = micros();
  } else {
    behind = false;
  }

  if (now - lastOuter > tOuter) {
    lastOuter = lastOuter + tOuter;
    //***********************BEGIN OUTER LOOP*******************************
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
      eStop = 0; //Also disable e-stop if tripped
    }
    if (channels[mode_ch]->getCh() > -300 && channels[mode_ch]->getCh() < 300) { // thumbstick mode, blue stick middle position
      mode = 1;
    }
    if (channels[mode_ch]->getCh() > 300) { // riding mode, blue stick down position
      mode = 2;
    }
    timer[5] = micros();

    // Debug related
    if (debugRiding) {
      Serial.println("***********");
      Serial.println(imu_channels[0]->getCh() - imu_zero[0]);
      Serial.println(imu_channels[1]->getCh() - imu_zero[1]);
      Serial.println(qd_d[0]);
      Serial.println(qd_d[1]);
      delay(20);
    }
    if (debugRx) {
      printChannels();
      delay(20);
    }
    if (debugTiming) {
      Serial.println();
      Serial.println();
      char buffer[40];
      sprintf(buffer, "After checking overtiming:        %i usec", timer[1] - timer[0]);
      Serial.println(buffer);
      sprintf(buffer, "After getting Rx inputs:          %i usec", timer[2] - timer[0]);
      Serial.println(buffer);
      sprintf(buffer, "After calculating kinematics:     %i usec", timer[3] - timer[0]);
      Serial.println(buffer);
      sprintf(buffer, "After sending CAN commands:       %i usec", timer[4] - timer[0]);
      Serial.println(buffer);
      sprintf(buffer, "After outer loop:                 %i usec", timer[5] - timer[0]);
      Serial.println(buffer);
      sprintf(buffer, "After Serial output (debug only): %i usec", micros() - timer[0]);
      Serial.println(buffer);
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
  delay(750);
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
