#include "Drive.h"
#include "DueCANLayer.h"
#include <math.h>
#include "utils.h"

extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte* cData, byte cDataLen);

Drive::Drive(double vMax, double aMax, double dRatio, double tInner, int len, int mot) {
  this->a = 0;            // Instantaneous acceleration
  this->v = 0;            // Instantaneous velocity
  this->vMax = vMax;      // Velocity limit, for both acceleration and velocity mode
  this->aMax = aMax;      // Acceleration limit, for both velocity (slewing) and acceleration mode
  this->dRatio = dRatio;  // scaling factor from physical rad/sec to erpms
  this->tInner = tInner;
  this->len = len;
  this->mot = mot;
  
    // Kinematics and constants calculations
  MaxDelRPM = aMax * tInner / 1000000.0;//need to change variables
}

// This function sends velocity commands to a Drive wheel, while abiding by acceleration limits
void Drive::slewVel(double vel, int ch, int rcLost){

  // used in loop()
  double delRPM = vel - this->v;
  
  if (abs(delRPM) < MaxDelRPM){
    this->v = vel;
  } else {
    this-> v= this->v + sign(delRPM) * MaxDelRPM;
  }
  this->v = constrain(this->v, -1*this->vMax, this->vMax);
  this->setVel(this->v,ch,rcLost);
}

// This function sends a Drive motor command - CAN layer, does not account for acceleration limits. Safety cutoff is done here
void Drive::setVel(double vel, int ch, int rcLost){//vel is in erpm

  // used in Drive()
  bool ext = true;
  
  for (int m = 0; m < len; m++) {
    this->cTxData1[len - m -1] = (int)vel >> 8 * m;
  }
  int idd = this->mot+1 | CAN_PACKET_SET_RPM << 8;  // 6/22/2021, added +1 because I indexed at 1, not 0
  if (ch > 400 && !rcLost){ // Only send motor if pot #5 is turned all the way to the right, and rc signal is present
    canTx(1, idd, ext, this->cTxData1, len);
  }
}

void Drive::setAcc(double acc, int ch, int rcLost){
  // used in loop()
  double delRPM = acc*tInner/1000000.0;
  
  if (abs(delRPM) < MaxDelRPM){
    this->v = this->v+delRPM;
  } else {
    this-> v= this->v + sign(delRPM) * MaxDelRPM;
  }
  this->v = constrain(this->v, -1*this->vMax, this->vMax);
  this->setVel(this->v,ch,rcLost);
}

double Drive::getVel(){
  return this->v;
}
