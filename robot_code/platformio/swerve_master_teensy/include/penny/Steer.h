#include <math.h>
#include "DueCANLayer.h"

#ifndef _STEER_
#define _STEER_


class Steer
{
  private:
    double v; // Current velocity of steer motors, degrees/second
    double vMax;  // Max angular velocity of steer motor, in motor frame, degrees/second
    double aMax;  // Max angular acceleration of steer motor, in motor frame, degrees/second^2
    double yRatio;
    double steer;   // current absolute angle of steer output stage, as tracked by trajectory planner
    double mPos;  // position of motor (pre gear stage) - always between 0 and 360°
    double tInner;
    unsigned char cTxData0[8] = {0xA3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // 0xA3 is for sending position commands
    int len;
    int mot;
    byte homing; // stores the current homing state: 0 = homed, 1 = just homed, 2 = not homed
    // CAN Rx vars
    byte cRxData[8];
    long rxlMsgID;
    bool rxbExtendedFormat;
    byte rxcDataLen;
    double currentPos;
  public:
    Steer(double vMax, double aMax, double yRatio, double tInner, int len, int mot);
    void yawTo(double ang, int ch, int rcLost);
    void motTo(double ang, int ch, int rcLost);
    void setYaw(double steer);
    void setMPos(double mPos);
    void setVel(double v);
    void setHoming(byte homing);
    double getYaw();
    double getMPos();
    byte getHoming();
}; 


#endif
