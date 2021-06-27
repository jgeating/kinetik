#include "DueCANLayer.h"
#include <math.h>
#include "Channel.h";
#include "Kinematics.h";
#include "utils.h";
#include "Yaw.h";
//#include "imu.h"; uncommented since we're no longer using BNO055
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
  30, // left switch, backwards = (+)
  31  // right switch, backwards = (+)
};

#define RADIUS_SWERVE_ASSEMBLY 0.25 // meters
#define YAW_GEAR_RATIO -18 // RMD-X6 Ratio = 8:1, pulley ratio = 72/32 = 2.25
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

//CAN stuff for steering motors
byte cTxData1[] = {0xA3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int idy = 1 || CAN_PACKET_SET_POS << 8; // Yaw ID
unsigned char buf[8];
int32_t ang = 0;

// Receiver stuff
int ir[] = {0, 0, 0, 0};               // status of IR sensor
short chs = 8;                      // number of channels to read from receiver
short chOff[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};      // Channel offsets (calibrate to find these)
Channel** channels = new Channel*[chs];
int rcTimeout = 100000;             // number of microseconds before receiver timeout is tripped - make sure it is a good bit longer than 2000 microseconds
bool rcLost = 1;                    // This is set to true if no receiver signal is received within the rcTimeout timeframe (microseconds)

//IMU and PWM comms stuff (uses channels)
short imu_chs = 2;
short imu_chOff[] = {1500,1500};
const int imu_ch_pins[] = {36, 37};//imu channel digital pins 27 is for x vel, 26 is for y vel
Channel** imu_channels = new Channel*[imu_chs];

// Robot state stuff
//looking from bottom, (+) to offset rotates wheel CCW
double irPos[] = {60, 292, 241, 91};     // absolute position if IR sensors, for calibrating position on startup, degrees. increasing rotates clockwise looking from the top
// //^^original values were -23,170,157,-10. Changed to correct for something

int irPin[] = {35, 34, 33, 32};           // pins that ir sensors are hooked up to
double mRPM[] = {0, 0, 0, 0};          // Speed of drive motors (pre gear stage). In eRPM, I think...
double yRatio = YAW_GEAR_RATIO;     // Yaw pulley stage ratio, >1
double dRatio = 1;                  // Drive ratio
int motPol[] = {1, 1, 1, 1};          // Used to switch motor direction depending on VESC configuration. Not implemented yet due to datatype issues. Just changing VESC parameters instead

// Kinematics
int nWheels = 4;
double velGain = 30; // 30 for humans driving
double yawGain = 60; // 60 for safe driving
Kinematics** kinematics = new Kinematics*[nWheels];

// Loop Timing Variables
double tInner = 1000;                 // Target length of inner loop controller, microseconds
double tOuter = 50000;                // Target length of outer (second) loop controller, microseconds
unsigned long lastInner = 0;          // last time counter of inner loop
unsigned long lastOuter = 0;          // last time counter of outer loop
unsigned long now = 0;                // variable to store current time, to avoid multiple calls to micros() function
bool behind = 0;                      // If Due can't keep up with loop rate, this bool will become true

Drive** drive = new Drive*[nWheels];
// Trajectory related
Yaw** yaw = new Yaw*[nWheels];
double aMaxYaw = 10000;                      // Max angular acceleration of yaw motor, in motor frame, degrees/second^2. Safe starting value: 5000
double wMaxYaw = 10000;                     // Max angular velocity of yaw motor, in motor frame, degrees/second. Safe starting value: 10000
double rampDist = 10.0;                  // Ramp distance is the distance at which the trajectory planner will apply full acceleration to reach the desired position, otherwise it will ramp acceleration linearly to 0. In degrees of the yaw stage
double aMaxRPM = 100000;                 // Max drive wheel acceleration in rpm/sec
double maxDelRPM;                        // max that wheel RPM can change in one control loop timestep
double rpmMax = 20000;                  // max RPM that motors are allowed to drive - proportional to max speed - will probably replace at some time, 100000 for safe driving
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
  Serial.print("attaching interrupts....");
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
     irPos[i] = irPos[i] + 90;    // Correcting for polar coordinate frame
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
  if (now - lastInner > tInner){
    if (behind){ // This means we can't keep up with the desired loop rate. Trip LED to indicate so
      digitalWrite(13, HIGH);
      lastInner = now;
    } else {
      digitalWrite(13, LOW);
      lastInner = lastInner + tInner;
      behind = true;
    }
    //***************BEGIN INNER LOOP**************************
    //printChannels();
    //pulseInChannels();
    //imu::Vector<3> angle = getAngle();

    if (mode == 0){ // in tele-op mode
      for (int k = 0; k<3; k++){  // calculate target x, y, w speeds in tele-op mode
          v[k] = constrain(channels[k+1]->getCh(), -500, 500);
          //Serial.print("   k:");
          //Serial.print(v[k]);
      }
      //Serial.println();
      for (int i = 0; i<nWheels; i++){
        //kinematics[i]->calc(channels[1]->getCh(), channels[2]->getCh(), channels[3]->getCh());
        kinematics[i]->calc((int)v[0], (int)v[1], (int)v[2]*sign(yRatio));
      }
    } else if (mode == 1){  // in riding mode
      //imu.updateIMU();
      //riding mode
      //Serial.print("Riding Mode");
      //Rules of thumb = 15,000 rpm is a relatively fast jog, 30,000 yaw rate is spinning pretty fast, but not insane
      int maxspeed = 300;
      a[0] = -map(imu_channels[0]->getCh(),-500,500,-maxspeed,maxspeed);
      a[1] = map(imu_channels[1]->getCh(),-500,500,-maxspeed,maxspeed);
      //for(int i = 0;i<2;i++){
//        if(a[i]>0){
//          a[i] = abs(maxspeed*(pow(a[i]/maxspeed,1.1)));
//        }else{
//          a[i] = -abs(maxspeed*(pow(a[i]/maxspeed,1.1)));
//        }
//      }
      for (int k = 0; k<3; k++){  // calculate target x, y, w speeds in tele-op mode
      //    delv[k] = a[k] * tInner / 1000000.0;
      //    v[k] = constrain(v[k]+delv[k], -100, 100);
        v[k] = constrain(a[k], -maxspeed, maxspeed);
      }
      //Serial.println();
      for (int i = 0; i<nWheels; i++){
        kinematics[i]->calc(v[0], v[1], v[2]);
      }
    }
    for (int i = 0; i<nWheels; i++){
      yaw[i]->yawTo(kinematics[i]->getTargetYaw(), channels[4]->getCh(), rcLost);
      delayMicroseconds(100); // Nasty bug where going from 3 motors to 4 per bus required a 100 us delay instead of 50 
      drive[i]->slewVel(kinematics[i]->getTargetRPM(),channels[4]->getCh(), rcLost);
      delayMicroseconds(100);
    }
  } else {
    behind = false;
  }

  if (now - lastOuter > tOuter){
    lastOuter = lastOuter + tOuter;

    //***********************BEGIN OUTER LOOP*******************************    
    checkRx();                          // check if receiver has been lost
    if (channels[5]->getCh() > 400){    // calibrate yaw motors
      //Serial.println("Calibrating motors");
      calMotor();
    }
    if (channels[6]->getCh() < -400&&channels[4]->getCh()<100) {  // calibrate force pads, only if steering motoors are off
      //Serial.println("Calibrating IMU");
      v[0] = 0;
      v[1] = 0;
      v[2] = 0;
      //imu.calibrateRange();
    }
    if (channels[7]->getCh() < -300) {  //default mode is tele-op
      //Serial.print(rcLost);
      //Serial.print("   ");
      //Serial.println("Tele-op mode");
      mode = 0;
      eStop = 0; //Also disable e-stop if tripped
      for (int i = 0; i < 3; i++){
        delvMax[i] = aMax[i] * tInner / 1000000.0;
      } 
    }
    if (channels[7]->getCh() >300) {  //for riding mode
      mode = 1;
      //Serial.println("riding mode");
      //imu.printReadouts();
    }
    if (channels[7]->getCh() > -300 && channels[7]->getCh() < 300){ //when switch is in center pos, set all velocities to zero to prep for riding
      v[0] = 0;
      v[1] = 0;
      v[2] = 0;
      //Serial.println("safe mode");
    }
    if (pads->fallDetected()) {
      eStop = 1;
      //Serial.println("Rider fall detected");
    }
  }
}

// *********************************************** HELPER FUNCTIONS **************************************************************

// ************** CALIBRATION **********************
void calMotor() {
  // variables only used here and in Yaw class: homing, Yaw, mPos, yRatio, pos

  // variables used here and other places: irPin
  
  //Serial.println("Starting calibration sequence...");
  
  for (int j = 0; j<nWheels; j++){
    yaw[j]->setHoming(2);               // set homing mode to true for all axes
    yaw[j]->motTo(0, channels[4]->getCh(), rcLost);
  }
  delay(1000);
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
        yaw[j]->setMPos((int)angTarget % 360);
        yaw[j]->setHoming(0);
      }
      if (yaw[j]->getHoming() == 0){    // position signal should persist, but motor should stop moving after cal marker is detected
        pos = yaw[j]->getMPos();
      }
      yaw[j]->motTo(pos, channels[4]->getCh(), rcLost);
    }
    delay(1);
    doneHoming = 1;
    for (int j = 0; j < nWheels; j++){
      doneHoming = doneHoming && yaw[j]->getHoming() == 0;
    }
    if (doneHoming == 1) break; // break from loop once all targets have been found
    

    // Hacky override to calibration, when ir sensors are broken, manually align to forwards within XX degrees
    /*
    int mPosStart[] = {109, 172, -57, -57};
    for (int j = 0; j<4; j++){
      yaw[j]->setYaw(-90);
      yaw[j]->setMPos(mPosStart[j]);
    } 
    */   
  }
}

// This function detects if a receiver signal has been received recently. Used for safety, etc. 
void checkRx(){
  if (micros() - channels[4]->getLastInterruptTime() > rcTimeout){
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
