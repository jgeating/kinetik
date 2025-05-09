#include "penny/Drive.h"
#include "DueCANLayer.h"
#include <math.h>
#include "shared/utils.h"
#include "Arduino.h"
#include "Swerve.h"

Drive::Drive(double vMax, double aMax, double dRatio, double tInner, int len, int mot, Drive::Type type)
{
  this->a = 0;           // Instantaneous acceleration
  this->v = 0;           // Instantaneous velocity
  this->vMax = vMax;     // Velocity limit, for both acceleration and velocity mode
  this->aMax = aMax;     // Acceleration limit, for both velocity (slewing) and acceleration mode
  this->dWheel = dWheel; // wheel diameter, m
  this->dRatio = dRatio; // scaling factor from physical rad/sec to erpms. erpm = pole pairs * rad/sec * 1/(2*pi) rev/rad * 60 sec/min
  this->tInner = tInner;
  this->len = len;
  this->mot = mot;
  this->type = type;

  // Kinematics and constants calculations
  MaxDelRPM = aMax * tInner / 1000000.0; // need to change variables

  if (this->type == Drive::Type::ODRIVE)
  {
    MaxDelRPM *= 1.0 / (this->odriveRatio);
    this->vMax *= 10 * this->odriveRatio;

    motors::drive[this->mot].enableWithClosedLoop();
    Serial.print("Setting motor to closed loop control mode: ");
    Serial.println(this->mot);

    delay(100);
  }
}

double bound(double value, double a, double b)
{
  return a < b ? constrain(value, a, b) : constrain(value, b, a);
}

// This function sends velocity commands to a Drive wheel, while abiding by acceleration limits. Input in m/s
void Drive::slewVel(double vel, int ch, int rcLost)
{

  double maxVel = sign(vel) * (abs(vel) + MaxDelRPM);
  double minVel = 0;
  this->v = constrain(bound(vel, minVel, maxVel), -1 * this->vMax, this->vMax);
  this->setVel(this->v, ch, rcLost);

  // used in loop()
  // double dv = vel - this->v;

  // if (abs(dv) < dv){
  //   this->v = vel;
  // } else {
  //   this-> v= this->v + sign(dv) * MaxDelRPM;
  // }
  // this->v = constrain(this->v, -1*this->vMax, this->vMax);
  // this->setVel(this->v,ch,rcLost);
}

// This function sends a Drive motor command - CAN layer, does not account for acceleration limits. Safety cutoff is done here
void Drive::setVel(double vel, int ch, int rcLost)
{ // vel is in erpm
  // vel = -vel;   // added 9/2/2023 because robot direction was reversed  bool eStop = !(ch > 400 && !rcLost);
  bool eStop = !(ch > 0 && !rcLost);

  // erpm to rpm
  float velocity = vel * this->odriveRatio;
  float torqueFF = 0.0;

  if (!eStop)
  { // Only send motor if safety channel is in the correct range, and rc signal is present
    motors::drive[this->mot].setVelocity(velocity * 100);
  }
  else
  { // Actively command zero velocity for ODrives. Otherwise, they will latch velocity. Might find a way to configure auto timeout in the future    
    motors::drive[this->mot].setVelocity(0);
  }
}

void Drive::setAcc(double acc, int ch, int rcLost)
{
  // used in loop()
  double dv = acc * tInner / 1000000.0;

  if (true || this->type == Drive::Type::VESC)
  {
    if (abs(dv) < MaxDelRPM)
    {
      this->v = this->v + dv;
    }
    else
    {
      this->v = this->v + sign(dv) * MaxDelRPM;
    }
    this->v = constrain(this->v, -1 * this->vMax, this->vMax);
    this->setVel(this->v, ch, rcLost);
  }
  else
  {
  }
}

double Drive::getVel()
{
  return this->v;
}
