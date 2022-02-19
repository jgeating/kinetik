#include "Yaw.h"
#include "DueCANLayer.h"
#include <math.h>
#include "utils.h"

extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte* cData, byte cDataLen);
extern byte canRx(byte cPort, long* lMsgID, bool* bExtendedFormat, byte* cData, byte* cDataLen);

Yaw::Yaw(double vMax, double aMax, double yRatio, double tInner, int len, int mot) {
  this->v = 0;
  this->vMax = vMax;
  this->aMax = aMax;
  this->yRatio = yRatio;
  this->yaw = 0;
  this->mPos = 0;
  this->tInner = tInner;
  this->len = len;
  this->mot = mot;
  this->homing = 0;
}

// This function traverses to a yaw position, while abiding by acceleration and velocity limits
void Yaw::yawTo(double ang, int ch, int rcLost){  

  // global variables only used here: v, aMax, vMax, yRatio

  // global variables used only here, init, and calibrate: yaw, mPos, 

  // global variables used everywhere: tInner

  // functions only used here and calibrate: motTo
  
  double delPos = ang - this->yaw;
  if (abs(delPos) > 180){
    while (abs(delPos) > 180){
      delPos = delPos + sign(delPos) * -360.0;
    }
  }
  double tStop = abs(this->v)/this->aMax;  // minimum time until motor can stop
  double minStop = abs(this->v)*tStop + this->aMax * tStop * tStop / 2; // minimum distance in which motor can stop  
  // double acc = constrain(abs(delPos)/rampDist, -1.0, 1.0) * aMax; // To prevent instabilities and numerical error buildup near zero
  // if (sign(delPos*v[mot]) == -1) v[mot] = 0;      // velocity should never be pushing away from setpoint, even if it breaks max accel limit
  if (abs(delPos) > minStop){
    this->v = this->v + sign(delPos) * this->aMax * this->tInner / 1000000.0;    // Speed up or coast (coasting done with constrain)
  } else {
    this->v = this->v - sign(this->v) * this->aMax * this->tInner / 1000000.0;
  }
  this->v = constrain(this->v, -1 * this->vMax, this->vMax);
  double delYaw = this->v * this->tInner / 1000000.0;
  this->yaw = (this->yaw + delYaw); 
  this->mPos = (this->mPos + delYaw * this->yRatio);
  this->motTo(this->mPos, ch, rcLost);
}

// This function sends a yaw motor command - CAN layer, does not account for acceleration limits. Safety cutoff is done here
void Yaw::motTo(double ang, int ch, int rcLost){
  //this->mPos = ang;
  // used in yawTo(), calMotor()
  bool ext = false;
  
  // for (int m = 0; m < this->len; m++) {
  //   this->cTxData0[this->len - m - 1] = (int)(ang * 1000000.0) >> 8*m;
  // }
  int32_t ang_int = (int32_t)(ang * 100);  // RMD-X6 takes angle over CAN as int32 in hundredths of degrees
  //ang = 0;
  this->cTxData0[4] = *((uint8_t *)(&ang_int));
  this->cTxData0[5] = *((uint8_t *)(&ang_int)+1);
  this->cTxData0[6] = *((uint8_t *)(&ang_int)+2);
  this->cTxData0[7] = *((uint8_t *)(&ang_int)+3);

  //int idy = this->mot | CAN_PACKET_SET_POS << 8; 
  int idy = 0x141 + this->mot;
  if (ch > -200 && !rcLost){ // Only send motor if ch [estop] is turned to center or farther, and rc signal is present
    canTx(    0,    idy,      ext, this->cTxData0,      8);
    //canTx(bus, CAN ID, ext bool,           data, length);
  }
}

void Yaw::setYaw(double yaw) {
  this->yaw = yaw;
}

void Yaw::setMPos(double mPos) {
  this->mPos = mPos;
}

void Yaw::setVel(double v) {
  this->v = v;
}

void Yaw::setHoming(byte homing) {
  this->homing = homing;
}

double Yaw::getYaw() {
  return this->yaw;
}

double Yaw::getMPos() {
  return this->mPos;
}

byte Yaw::getHoming() {
  return this->homing;
}
