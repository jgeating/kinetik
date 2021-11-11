#include "Planner.h"
#include <cmath>

Planner::Planner(double dt, double qd_x_max, double qd_y_max, double qd_z_max, double qdd_x_max, double qdd_y_max, double qdd_z_max, double x_dead, double y_dead, double z_dead) {
  this->dt = dt;      // length of time step, microseconds

  this->qd_x_d = 0;   // desired x velocity, m/s
  this->qd_y_d = 0;   // desired y velocity, m/s
  this->qd_z_d = 0;   // desired yaw velocity, rad/sec
  this->q_z_d = 0;    // desired heading, rad

  this->qd_x = 0;     // current x velocity, m/s
  this->qd_y = 0;     // current y velocity, m/s
  this->qd_z = 0;     // current yaw velocity, rad/sec
  this->q_z = 0;      // current orientation, rad

  this->qd_x_max = qd_x_max;   // max x vel allowed, m/s 
  this->qd_y_max = qd_y_max;   // max y vel allowed, m/s 
  this->qd_z_max = qd_z_max;   // max yaw vel allowed, rad/s
  this->qdd_x_max = qdd_x_max; // max x acceleration allowed, m/s^2 
  this->qdd_y_max = qdd_y_max; // max y acceleration allowed, m/s^2 
  this->qdd_z_max = qdd_z_max; // max yaw acceleration allowed, rad/s^2

  this->x_dead = x_dead;      // dead zone in x axis, m/s. Robot will not move if desired is slower than this
  this->y_dead = y_dead;      // dead zone in y axis, m/s. Robot will not move if desired is slower than this
  this->z_dead = z_dead;      // dead zone in w axis, m/s. Robot will not move if desired is slower than this

  this->del_qd_x_max = qdd_x_max * dt/1000000.0;
  this->del_qd_y_max = qdd_y_max * dt/1000000.0;
  this->del_qd_z_max = qdd_z_max * dt/1000000.0;
}

void Planner::calc(double x_in, double y_in, double z_in) { // inputs are in SI units, desired  
  double del_qd = 0;
  
  this->qd_x_d = x_in;                                                                  // Set desireds to inputs. All units in m/s or rad/sec
  if (std::abs(this->qd_x_d) < this->x_dead){ this->qd_x_d = 0; }                       // set desired to zero if within deadband
  del_qd = qd_x_d - qd_x;
  if (std::abs(del_qd) > del_qd_x_max){
    this->qd_x = this->qd_x + this->sign(del_qd) * del_qd_x_max; // accelerate as much as allowed  
  } else {
    this->qd_x = qd_x_d;
  }
  this->qd_x = this->lim(this->qd_x, -1*this->qd_x_max, this->qd_x_max);                // constrain to max vel

  this->qd_y_d = y_in;                                                                  // Set desireds to inputs. All units in m/s or rad/sec
  if (std::abs(this->qd_y_d) < this->y_dead){ this->qd_y_d = 0; }                       // set desired to zero if within deadband
  del_qd = qd_y_d - qd_y;
  if (std::abs(del_qd) > del_qd_y_max){
    this->qd_y = this->qd_y + this->sign(del_qd) * del_qd_y_max; // accelerate as much as allowed  
  } else {
    this->qd_y = qd_y_d;
  }
  this->qd_y = this->lim(this->qd_y, -1*this->qd_y_max, this->qd_y_max);                  // constrain to max vel

  this->qd_z_d = z_in;                                                                  // Set desireds to inputs. All units in m/s or rad/sec
  if (std::abs(this->qd_z_d) < this->z_dead){ this->qd_z_d = 0; }                       // set desired to zero if within deadband
  del_qd = qd_z_d - qd_z;
  if (std::abs(del_qd) > del_qd_z_max){
    this->qd_z = this->qd_z + this->sign(del_qd) * del_qd_z_max; // accelerate as much as allowed  
  } else {
    this->qd_z = qd_z_d;
  }
  this->qd_z = this->lim(this->qd_z, -1*this->qd_z_max, this->qd_z_max);                  // constrain to max vel
}

double Planner::getTargetVX() {
  return this->qd_x;
}

double Planner::getTargetVY() {
  return this->qd_y;
}

double Planner::getTargetVZ() {
  return this->qd_z;
}

int Planner::sign(double x) {
  //return (x > 0) - (x < 0);
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 0;
}

double Planner::lim(double x, double a, double b) {
    if(x < a) {
        return a;
    }
    else if(b < x) {
        return b;
    }
    else
        return x;
}
