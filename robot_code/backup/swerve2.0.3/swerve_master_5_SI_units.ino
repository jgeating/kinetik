#include "DueCANLayer.h"
#include <math.h>
#include "Channel.h";
#include "Kinematics.h";
#include "utils.h";
#include "Yaw.h";
#include "Pads.h";
#include "Drive.h"
//#include "IMU_Controller.h"
#include "Wire.h"

const int CHANNEL_PIN[] = {
  24,  // left stick vertical, forward = (+)
  25,  // right stick horizontal, R = (+)
  26,  // right stick vertical, forward = (+)
  27,  // left stick horizontal, R = (+)
  28,  // left knob, CW = (+)
  29,  // right knob, CW = (+)
  30, // left switch, backwards = (-)
  31  // right switch, backwards = (-)
};

#define RADIUS_SWERVE_ASSEMBLY 0.25 // meters
#define YAW_GEAR_RATIO -18 // RMD-X6 Ratio = 9:1, pulley ratio = 72/32 = 2.25
#define DEAD_ZONE 30

void displayRCValues(void);

extern byte canInit(byte cPort, long lBaudRate);
extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte* cData, byte cDataLen);
extern byte canRx(byte cPort, long* lMsgID, bool* bExtendedFormat, byte* cData, byte* cDataLen);

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
int driveCanDelay = 100; // # of microseconds to delay sending CAN for

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
short imu_pulse_range[] = {(imu_max_pulse[0]-imu_min_pulse[0]), (imu_max_pulse[1]-imu_min_pulse[1])};
const int imu_ch_pins[] = {2, 3}; //imu channel digital pins
Channel** imu_channels = new Channel*[imu_chs];
double orientation[] = {0, 0};  //stores orientation in degrees
double pulse2deg[] = {360.0/(double)imu_pulse_range[0], 360.0/(double)imu_pulse_range[1]};

// Robot state stuff
int ir[] = {0, 0, 0, 0};            // status of IR sensor
//looking from bottom, (+) to offset rotates wheel CCW
double irPos[] = {254, 84, 73, 257};     // absolute position if IR sensors, for calibrating position on startup, degrees. increasing rotates clockwise looking from the top
// 8/8/2021 values were 158, 347, 342, 171
int irPin[] = {35, 34, 33, 32};       // pins that ir sensors are hooked up to
double mRPM[] = {0, 0, 0, 0};         // Speed of drive motors (pre gear stage). In eRPM, I think...
double yRatio = YAW_GEAR_RATIO;       // Yaw pulley stage ratio, >1
double dRatio = 1;                    // Drive ratio
int motPol[] = {1, 1, 1, 1};          // Used to switch motor direction depending on VESC configuration. Not implemented yet due to datatype issues. Just changing VESC parameters instead

// Kinematics
int nWheels = 4;
int pole_pairs = 7;   // number of pole pairs in hub motors. Assuming 7 pole pairs from research. did not actually measure. Used for converting erpm to rpm, to calculate real velocities. 
double velGain = 80; // 30 for humans driving, emperically detemined to align control range with max speed
double yawGain = 200; // 60 for safe driving, emperically detemined
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

Drive** drive = new Drive*[nWheels];
// Trajectory related
Yaw** yaw = new Yaw*[nWheels];
double aMaxYaw = 10000;                      // Max angular acceleration of yaw motor, in motor frame, degrees/second^2. Safe starting value: 5000
double wMaxYaw = 10000;                     // Max angular velocity of yaw motor, in motor frame, degrees/second. Safe starting value: 10000
double rampDist = 10.0;                  // Ramp distance is the distance at which the trajectory planner will apply full acceleration to reach the desired position, otherwise it will ramp acceleration linearly to 0. In degrees of the yaw stage
double aMaxRPM = 100000;                 // Max drive wheel acceleration in rpm/sec
double maxDelRPM;                        // max that wheel RPM can change in one control loop timestep
double rpmMax = 40000;                  // max RPM that motors are allowed to drive - proportional to max speed - will probably replace at some time, 100000 for safe driving
double xYawCoeff[] = {0, 0, 0, 0};
double yYawCoeff[] = {0, 0, 0, 0};
int doneHoming = 0;                      // Used to determine when calibration sequence is finished. 1 = finished. 
//sensors_event_t event; 

// Robot level trajectory/control
//double aMax[] = {200000, 200000, 500000}; // Max x, y, w acceleration of bot CG
//double vMax[] = {100000, 100000, 100000}; // Max velocity of bot CG, in wheel rpm's (until everything gets converted to SI)
double aMax[] {50000, 50000, 50000};
double vMax[] = {1000, 1000, 200}; //{20000, 20000, 10000}
double v[] = {0, 0, 0};      // current robot x, y, w velocity
double delv[] = {0, 0, 0};   // delta velocity per tick
double a[] = {0, 0, 0};      // current robot x, y, w acceleration only for riding mode, otherwise a constant max value is used, and robot is in velocity control
double delvMax[] = {0, 0, 0};   // instantaneous x, y, w max velocity dt per tick, for calculations

// Force Sensor Variables
// Used pins: Analog                0,  1,  2,  3,             4,  5,  6,  7
// In the above order, Right Side: FL, FR, BR, BL, Left Side: BR, BL, FR, FL
// Order for programming:       R: FL, FR, BL, BR,         L: FL, FR, BL, BR
int forcepins[] =                 {A0, A1, A3, A2,            A7, A6, A5, A4};    // Select the input pins for the load cells
int forcezeros[] {1995, 1852, 2229, 2348, 2152, 2081, 2007, 2305};                // Zero force values with pads on top of sensors - in forcepins[] order
double lccapacity = 50; // Load cell capacity, kg
int anRes = 12;                           // analogRead resolution, bits
double lcgain = lccapacity / (pow(2, anRes)/2); // Scaling constant to convert from analog value to kilogram-force on load cell
double xgain = 100.0;
double ygain = 100.0;
double zgain = 100.0;
double xyMax = 50; // Max lateral velocity, in rpm - need to calculate everything into true SI units
double wMax = 30;  // Max spin rate, in arbitrary units
double weightthresh = 30;  //min weight to not enable estop
Pads* pads = new Pads(forcepins, forcezeros, xgain, ygain, zgain, xyMax, wMax, weightthresh);
int mode = 1;   //0 = RC mode (teleop), 1 = weight control mode
int eStop = 0;  // e-stop variable for safety

// Debug related - generally outputs serial information, which will slow down loop speeds. 0 = don't print out to serial, 1 = print information to serial 
bool debugRx = 0;       // whether or not to debug receiver
bool debugTiming = 0;   // whether or not to debug timing, look at loop lengths, etc. 
bool debugRiding = 0;

void setup()
{
  // Setup hardware
  Serial.begin(115200);
  Serial.println("Starting up...");
  analogReadResolution(anRes);

  Serial.println("Initializing CAN and pinmodes...");
  if(canInit(0, CAN_BPS_1000K) == CAN_OK)
    Serial.print("CAN0: Initialized Successfully.\n\r");
  else
    Serial.print("CAN0: Initialization Failed.\n\r");
  if(canInit(1, CAN_BPS_500K) == CAN_OK)
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
  
  Serial.println("RC interrrupts initialized. Initializing IMU PWM Comms interrupts...");
  
  for (int i = 0; i < imu_chs; i++) {
    imu_channels[i] = new Channel(imu_ch_pins[i], imu_chOff[i]);
    Serial.print("Channel ");
    Serial.print(i);
    Serial.print("created.");
  }
  Serial.println();
  Serial.print("Attaching IMU interrupts....");
  attachInterrupt(imu_channels[0]->getPin(), imu_calcCh1, CHANGE);
  attachInterrupt(imu_channels[1]->getPin(), imu_calcCh2, CHANGE);
  Serial.println("IMU PWM Comms interrupts initialized. Calculating Kinematics and constants....");
  // Kinematics and constants calculations
   maxDelRPM = aMaxRPM * tInner / 1000000.0;
   for (int i = 0; i < 3; i++){
    delvMax[i] = aMax[i] * tInner / 1000000.0;
   }

   Serial.println("Initializing yaw, drive and kinematics objects.");
   for (int i = 0; i < nWheels; i++){
     irPos[i] = irPos[i];    // Correcting for polar coordinate frame
     yaw[i] = new Yaw(wMaxYaw, aMaxYaw, yRatio, tInner, len, i);
     drive[i] = new Drive(rpmMax,aMaxRPM,dRatio, tInner,len, i);
     kinematics[i] = new Kinematics(velGain, yawGain, RADIUS_SWERVE_ASSEMBLY, DEAD_ZONE, i);
  } 
  //pads->calibrate();
  //Serial.println("Pads Calibrated");
  Serial.println("Startup Complete.");
}

void printChannels(){
  Serial.println("");
  Serial.println("---------");
  for(int i = 0;i<chs;i++)
    {
    Serial.print("Pin: ");
    Serial.print(CHANNEL_PIN[i]);
    Serial.print(", ");
    Serial.print(i);
    Serial.print(":");
    Serial.println(channels[i]->getCh());
    }
}

void pulseInChannels(){
   for(int i = 1;i<chs;i++)
    {
    //Serial.print(CHANNEL_PIN[i]);
    Serial.print(i);
    Serial.print(":");
    Serial.print(pulseIn(CHANNEL_PIN[i],HIGH));
    Serial.print(" ");
    }
    Serial.println("");
}

void loop()
{
  now = micros();
  timer[0] = now;
  if (now - lastInner > tInner){  
    if (behind){ // This means we can't keep up with the desired loop rate. Trip LED to indicate so
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

    //mode = 0; // temp override for testing without Tx 
    if (mode == 0){ // in tele-op mode
      for (int k = 0; k<3; k++){  // calculate target x, y, w speeds in tele-op mode
          v[k] = constrain(channels[k+1]->getCh(), -500, 500);
          //Serial.print("   k:");
          //Serial.print(v[k]);
      }
      timer[2] = micros();
      for (int i = 0; i<nWheels; i++){
        kinematics[i]->calc((int)v[0], (int)v[1], (int)v[2]*sign(yRatio));
      }
      timer[3] = micros();
    } else if (mode == 1){  // in riding mode
      double again = 1.0;
      double maxspeed = 250;
      orientation[0] = (imu_channels[0]->getCh() - imu_zero[0]) * pulse2deg[0];
      orientation[1] = (imu_channels[1]->getCh() - imu_zero[1]) * pulse2deg[1];
      
      while abs(orientation[0] > 180){
        if (orientation[0] > 180) {orientation[0] = orientation[0] - 360;}
        if (orientation[0] < -180) {orientation[0] = orientation[0] + 360;}
      }
      while abs(orientation[1] > 180){
        if (orientation[1] > 180) {orientation[1] = orientation[1] - 360;}
        if (orientation[1] < -180) {orientation[1] = orientation[1] + 360;}
      }
      a[0] = orientation[0]/45;
      a[1] = orientation[1]/45;
      for (int i = 0; i < 2; i++){
        v[i] = v[i] + (a[i] * again * tInner/1000000 * maxspeed);
        v[i] = constrain(v[i], -maxspeed, maxspeed);
      }
      v[2] = constrain(channels[2+1]->getCh(), -500, 500);
      
      for (int i = 0; i<nWheels; i++){
        kinematics[i]->calc(v[0], v[1], v[2]);
      }
    }
    for (int i = 0; i<nWheels; i++){
      yaw[i]->yawTo(kinematics[i]->getTargetYaw(), channels[estop_ch]->getCh(), rcLost);
      delayMicroseconds(steerCanDelay); // Nasty bug where going from 3 motors to 4 per bus required a 100 us delay instead of 50 
      drive[i]->slewVel(kinematics[i]->getTargetRPM(),channels[estop_ch]->getCh(), rcLost);
      delayMicroseconds(driveCanDelay);
    }
    timer[4] = micros();
  } else {
    behind = false;
  }

  if (now - lastOuter > tOuter){
    lastOuter = lastOuter + tOuter;
    //***********************BEGIN OUTER LOOP*******************************    
    checkRx();                          // check if receiver has been lost

    // Set gains
    for (int i = 0; i<nWheels; i++){
      double temp = mapDouble(channels[4]->getCh(), -500, 500, 0, 1);
      kinematics[i]->setVelGain(temp * velGain);
      kinematics[i]->setYawGain(mapDouble(channels[0]->getCh(), -500, 500, 0, temp * yawGain)); // Set yaw gain, premultiplied by velGain
    }

    // Check channels
    if (channels[5]->getCh() > 400){ calMotor(); } // Calibrate motors
    if (channels[estop_ch]->getCh() < -400&&channels[estop_ch]->getCh()<100) {  // calibrate force pads, only if steering motoors are off
      v[0] = 0;
      v[1] = 0;
      v[2] = 0;
      //imu.calibrateRange();
    }
    if (channels[mode_ch]->getCh() < -300) {  //default mode is tele-op
      mode = 0;
      eStop = 0; //Also disable e-stop if tripped
      for (int i = 0; i < 3; i++){
        delvMax[i] = aMax[i] * tInner / 1000000.0;
      }
    }
    if (channels[mode_ch]->getCh() >300) {  //for riding mode
      mode = 1;
      //Serial.println("riding mode");
      //imu.printReadouts();
    }
    if (channels[mode_ch]->getCh() > -300 && channels[mode_ch]->getCh() < 300){ //when switch is in center pos, set all velocities to zero to prep for riding
      v[0] = 0;
      v[1] = 0;
      v[2] = 0;
    }
    if (pads->fallDetected()) {
      eStop = 1;
      //Serial.println("Rider fall detected");
    }
    timer[5] = micros();
    if (debugRiding){
        Serial.println("***********");
        Serial.println(imu_channels[0]->getCh() - imu_zero[0]);
        Serial.println(imu_channels[1]->getCh() - imu_zero[1]);
        Serial.println(a[0]);
        Serial.println(a[1]);
        Serial.println(v[0]);
        Serial.println(v[1]);
        delay(20);
        }
    if (debugRx){
      printChannels();
      //pulseInChannels();
      delay(20);
    }
    if (debugTiming){
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
  for (int j = 0; j<nWheels; j++){
    yaw[j]->setHoming(2);               // set homing mode to true for all axes
    yaw[j]->motTo(0, channels[estop_ch]->getCh(), rcLost);
  }
  delay(750);
  double fineTune = 1;    // to step in less than 1 deg increments - this is the ratio (0.2 would be in 0.2 degree increments
  double angTarget = 0;
  for (int i = 0; i < (int)(360 * abs(yRatio) * 1.5 / fineTune); i++){
    checkRx();
    for (int j = 0; j < nWheels; j++){
      angTarget = i * fineTune;
      pos = i * fineTune;
      if (yaw[j]->getHoming() == 2 && digitalRead(irPin[j]) == 1){  // calibration started with IR triggered
        yaw[j]->setHoming(1);
      }
      if (yaw[j]->getHoming() == 1 && digitalRead(irPin[j]) == 0){ // Hit target fresh
        yaw[j]->setYaw(irPos[j]);
        yaw[j]->setMPos(pos);
        yaw[j]->setHoming(0);
      }
      if (yaw[j]->getHoming() == 0){    // position signal should persist, but motor should stop moving after cal marker is detected
        pos = yaw[j]->getMPos();
      }
      yaw[j]->motTo(pos, channels[estop_ch]->getCh(), rcLost);
    }
    delayMicroseconds(800);
    doneHoming = 1;
    for (int j = 0; j < nWheels; j++){
      doneHoming = doneHoming && yaw[j]->getHoming() == 0;
    }
    if (doneHoming == 1) {
      calibrated = 1; // set calibration flag to true
    }
    if (doneHoming == 1) break; // break from loop once all targets have been found
  }
}

double mapDouble(double x, double min_in, double max_in, double min_out, double max_out){
  double ret = (x - min_in)/(max_in - min_in);
  ret = ret * (max_out - min_out) + min_out;
  return ret;
}

// This function detects if a receiver signal has been received recently. Used for safety, etc. 
void checkRx(){
  if (micros() - channels[estop_ch]->getLastInterruptTime() > rcTimeout){
    rcLost = 1;
  } else {
    rcLost = 0;
  }
}
// ************************* CAN RECEIVER
void rxMsg(){
  if(canRx(0, &lMsgID, &bExtendedFormat, &cRxData[0], &cDataLen) == CAN_OK)
  {
    rxData = 0;
    for (int i = 0; i < len; i++){
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

// ***********************2.4 GHz RECEIVER  FUNCTIONS
void imu_calcCh1() {
    imu_channels[0]->calc();
}
void imu_calcCh2() {
    imu_channels[1]->calc();
}
