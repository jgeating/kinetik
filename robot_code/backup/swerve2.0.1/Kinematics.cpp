#include "Kinematics.h"
#include <math.h>

Kinematics::Kinematics(double velGain, double yawGain, double RADIUS_SWERVE_ASSEMBLY, int DEAD_ZONE, int wheelIndex) {
  this->velGain = velGain;
  this->yawGain = yawGain;
  this->DEAD_ZONE = DEAD_ZONE;
  this->yYawCoeff = RADIUS_SWERVE_ASSEMBLY *  cos((2.0/4.0)*M_PI * wheelIndex - (M_PI/4.0));
  this->xYawCoeff = RADIUS_SWERVE_ASSEMBLY *  sin((2.0/4.0)*M_PI * wheelIndex - (M_PI/4.0));
  this->targetYaw = 0;
  this->targetRPM = 0;
}

void Kinematics::calc(int xChannel, int yChannel, int zChannel) {
  double velocity_x = xChannel * this->velGain;
  double velocity_y = yChannel * this->velGain;
  double omega = zChannel * yawGain;
  
  double wheel_velocity_x = velocity_x + (omega * this->xYawCoeff);
  double wheel_velocity_y = velocity_y + (omega * this->yYawCoeff);
  double vel = sqrt(wheel_velocity_x * wheel_velocity_x + wheel_velocity_y * wheel_velocity_y);
  if (vel > this->DEAD_ZONE * this->velGain){
    double wheel_angle = atan2(wheel_velocity_y, wheel_velocity_x)*180.0/M_PI;
    this->targetYaw = wheel_angle;
  } else {
    vel = 0;
  }
  this->targetRPM = vel;
}

double Kinematics::getTargetYaw() {
  return this->targetYaw;
}

double Kinematics::getTargetRPM() {
  return this->targetRPM;
}
