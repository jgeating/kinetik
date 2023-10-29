#include "Kinematics.h"
#include <math.h>

Kinematics::Kinematics(double RADIUS_SWERVE_ASSEMBLY, int DEAD_ZONE, int wheelIndex) {
  this->DEAD_ZONE = DEAD_ZONE;
  this->yYawCoeff = cos((2.0/4.0)*M_PI * wheelIndex - (M_PI/4.0));
  this->xYawCoeff = sin((2.0/4.0)*M_PI * wheelIndex - (M_PI/4.0));
  this->targetYaw = 0;
  this->targetVel = 0;
  this->r = RADIUS_SWERVE_ASSEMBLY;
}

void Kinematics::calc(double vx, double vy, double vz) {
  double qd_x = vx;
  double qd_y = vy;
  double qd_z = vz;
  
  double wheel_velocity_x = qd_x + (r * qd_z * this->xYawCoeff);
  double wheel_velocity_y = qd_y + (r * qd_z * this->yYawCoeff);
  double vel = sqrt(wheel_velocity_x * wheel_velocity_x + wheel_velocity_y * wheel_velocity_y);
  if (vel > this->DEAD_ZONE){
    double wheel_angle = atan2(wheel_velocity_y, wheel_velocity_x)*180.0/M_PI;
    this->targetYaw = wheel_angle;
  } else {
    vel = 0;
  }
  this->targetVel = vel;
}

double Kinematics::getTargetYaw() {
  return this->targetYaw;
}

double Kinematics::getTargetVel() {
  return this->targetVel;
}
