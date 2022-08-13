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
  38,  // ch. 1 left stick vertical, forward = (+)
  40,  // ch. 2 right stick horizontal, R = (+)
  42,  // ch. 3 right stick vertical, forward = (+)
  44,  // ch. 4 left stick horizontal, R = (+)
  46,  // ch. 5 left knob, CW = (+)
  48,  // ch. 6 right knob, CW = (+)
  50,  // ch. 7 left switch, backwards = (-)
  52   // ch. 8 right switch, backwards = (-)
};

// PWM/Receiver stuff
short chs = 8;                      // number of channels to read from receiver
short chOff[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};      // Channel offsets (calibrate to find these)
Channel** channels = new Channel*[chs];
int rcTimeout = 100000;             // number of microseconds before receiver timeout is tripped - make sure it is a good bit longer than 2000 microseconds
bool rcLost = 1;                    // This is set to true if no receiver signal is received within the rcTimeout timeframe (microseconds)
int estop_ch = 6;                   // which Rx channel is used for motor enabling/SW e-stop. 0 index
int mode_ch = 7;                    // which Rx channel is used for setting mode

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // RC PWM interrupt setup
  for (int i = 0; i < chs; i++) { channels[i] = new Channel(CHANNEL_PIN[i], chOff[i]); }
  attachInterrupt(channels[0]->getPin(), calcCh1, CHANGE);
  attachInterrupt(channels[1]->getPin(), calcCh2, CHANGE);
  attachInterrupt(channels[2]->getPin(), calcCh3, CHANGE);
  attachInterrupt(channels[3]->getPin(), calcCh4, CHANGE);
  attachInterrupt(channels[4]->getPin(), calcCh5, CHANGE);
  attachInterrupt(channels[5]->getPin(), calcCh6, CHANGE);
  attachInterrupt(channels[6]->getPin(), calcCh7, CHANGE);
  attachInterrupt(channels[7]->getPin(), calcCh8, CHANGE);
  Serial.println("RC interrrupts initialized. Setting up kinematics and trajectory planning objects...");

}

void loop() {
  for (int i = 0; i < 8; i++){
    Serial.println(channels[i]->getCh());
  }
  delay(100);
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
