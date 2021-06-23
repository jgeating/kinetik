#include "DueCANLayer.h"
#include <math.h>

#ifndef _YAW_
#define _YAW_


class Yaw
{
  private:
    double v; // Current velocity of yaw motors, degrees/second
    double vMax;  // Max angular velocity of yaw motor, in motor frame, degrees/second
    double aMax;  // Max angular acceleration of yaw motor, in motor frame, degrees/second^2
    double yRatio;
    double yaw;   // current absolute angle of yaw output stage, as tracked by trajectory planner
    double mPos;  // position of motor (pre gear stage) - always between 0 and 360°
    double tInner;
    byte cTxData0[8] = {0xA3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int len;
    int mot;
    byte homing; // stores the current homing state: 0 = homed, 1 = just homed, 2 = not homed
  public:
    Yaw(double vMax, double aMax, double yRatio, double tInner, int len, int mot);
    void yawTo(double ang, int ch, int rcLost);
    void motTo(double ang, int ch, int rcLost);
    void setYaw(double yaw);
    void setMPos(double mPos);
    void setHoming(byte homing);
    double getYaw();
    double getMPos();
    byte getHoming();
}; 


#endif
