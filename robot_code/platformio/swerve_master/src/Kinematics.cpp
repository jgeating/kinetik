#include "Kinematics.h"
#include <math.h>

Kinematics::Kinematics(double RADIUS_SWERVE_ASSEMBLY, int DEAD_ZONE, int wheelIndex) {
  this->DEAD_ZONE = DEAD_ZONE;
  this->ySteerCoeff = cos((2.0/4.0)*M_PI * wheelIndex - (M_PI/4.0));
  this->xSteerCoeff = sin((2.0/4.0)*M_PI * wheelIndex - (M_PI/4.0));
  this->targetSteer = 0;
  this->targetVel = 0;
  this->r = RADIUS_SWERVE_ASSEMBLY;
}

void Kinematics::calc(double vx, double vy, double vz) {
  double qd_x = vx;
  double qd_y = vy;
  double qd_z = vz;
  
  double wheel_velocity_x = qd_x + (r * qd_z * this->xSteerCoeff);
  double wheel_velocity_y = qd_y + (r * qd_z * this->ySteerCoeff);
  double vel = sqrt(wheel_velocity_x * wheel_velocity_x + wheel_velocity_y * wheel_velocity_y);
  if (vel > this->DEAD_ZONE){
    this->targetSteer = atan2(wheel_velocity_y, wheel_velocity_x);
  } else {
    vel = 0;
  }
  this->targetVel = vel;
}

double Kinematics::getTargetSteer() {
  return this->targetSteer;
}

double Kinematics::getTargetVel() {
  return this->targetVel;
}
