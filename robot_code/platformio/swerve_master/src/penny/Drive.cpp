#include "penny/Drive.h"
#include "DueCANLayer.h"
#include <math.h>
#include "shared/utils.h"

extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte *cData, byte cDataLen);

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

    int idd = this->mot << 5 | 0x07; // idd is the id of the CAN packet
    bool ext = false;
    byte odrive_data[4] = {0x08, 0x00, 0x00, 0x00};

    canTx(1, idd, ext, odrive_data, sizeof(odrive_data));
    delay(1000);
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
  // vel = -vel;   // added 9/2/2023 because robot direction was reversed

  // used in Drive()
  bool ext = true;
  if (this->type == Drive::Type::VESC)
  {
    for (int m = 0; m < len; m++)
    {
      this->cTxData1[len - m - 1] = (int)(vel * dRatio) >> 8 * m;
    }
    int idd = this->mot | CAN_PACKET_SET_RPM << 8;
    if (ch > 400 && !rcLost)
    { // Only send motor if safety channel is in the correct range, and rc signal is present
      canTx(1, idd, ext, this->cTxData1, len);
    }
  }
  else
  {

    bool eStop = !(ch > 400 && !rcLost);

    // erpm to rpm
    float velocity = eStop ? 0 : vel * this->odriveRatio;
    float torqueFF = 0.0;

    byte odrive_data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    memcpy(odrive_data, &velocity, sizeof(velocity));
    memcpy(odrive_data + sizeof(velocity), &torqueFF, sizeof(torqueFF));

    int idd = this->mot << 5 | 0x0d;

    // Serial.println("vel:");
    // Serial.println(velocity);
    if (ch > 400 && !rcLost)
    { // Only send motor if safety channel is in the correct range, and rc signal is present
      canTx(1, idd, false, odrive_data, sizeof(odrive_data));
    }
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
