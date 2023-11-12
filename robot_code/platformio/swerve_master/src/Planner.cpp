#include "Planner.h"
#include <cmath>
// #include <math.h>

// planner = new Planner(tInner,      qd_max[0],       qd_max[1],       qd_max[2],       qdd_max[0],       qdd_max[1],       qdd_max[2],         dz[0],         dz[1],         dz[2],     mode,        maxLean);

Planner::Planner(double tInner, double qd_x_max, double qd_y_max, double qd_z_max, double qdd_x_max, double qdd_y_max, double qdd_z_max, double x_dead, double y_dead, double z_dead, int mode) {
  this->mode = mode;              // mode for what inputs/control algorithm to use
  this->dt = tInner / 1000000.0;  // length of time step, microseconds

  double in0[3];  // zeros for input angles from e.g. IMU(s), rad
  this->in0[0] = 0;
  this->in0[1] = 0;
  this->in0[2] = 0;

  double input[3];  // input for various control modes, units dependent on control mode
  this->input[0] = 0;
  this->input[1] = 0;
  this->input[2] = 0;

  double q[3] = { 0, 0, 0 };  // Current position/orientation, x/y/yaw, m or rad
  this->q[0] = 0;
  this->q[1] = 0;
  this->q[2] = 0;

  double qd[3];  // Current velocity, m/s or rad/s
  this->qd[0] = 0;
  this->qd[1] = 0;
  this->qd[2] = 0;

  double qdd[3];  // current acceleration, m/s^2 or rad/s^2
  this->qdd[0] = 0;
  this->qdd[1] = 0;
  this->qdd[2] = 0;

  double q_d[3];  // desired position, m or rad
  this->q_d[0] = 0;
  this->q_d[1] = 0;
  this->q_d[2] = 0;

  double qd_d[3];  // desired velocity, m/s or rad/s
  this->qd_d[0] = 0;
  this->qd_d[1] = 0;
  this->qd_d[2] = 0;

  double qdd_d[3];  // Desired acceleration, m/s^2 or rad/s^2
  this->qdd_d[0] = 0;
  this->qdd_d[1] = 0;
  this->qdd_d[2] = 0;

  double qd_max[3];  // max velocity allowed, m/s or rad/s
  this->qd_max[0] = qd_x_max;
  this->qd_max[1] = qd_y_max;
  this->qd_max[2] = qd_z_max;

  double qdd_max[3];  // max acceleration allowed, m/s^2 or rad/s^2
  this->qdd_max[0] = qdd_x_max;
  this->qdd_max[1] = qdd_y_max;
  this->qdd_max[2] = qdd_z_max;

  double dband[3];  // dead band zone, in m/s or rad/s. robot will not move if desired is slower than this
  this->dband[0] = x_dead;
  this->dband[1] = y_dead;
  this->dband[2] = z_dead;

  double del_qd_max[3];  // max amount velocity can change by in one dt
  this->del_qd_max[0] = this->qdd_max[0] * this->dt;
  this->del_qd_max[1] = this->qdd_max[1] * this->dt;
  this->del_qd_max[2] = this->qdd_max[2] * this->dt;
}

int Planner::plan(double x_in, double y_in, double z_in) {
  this->input[0] = x_in;
  this->input[1] = y_in;
  this->input[2] = z_in;

  switch (this->mode) {

    // ********** TELE-OP MODE *************
    case 0:  // TELE-OP MODE
      for (int i = 0; i < 3; i++) {
        this->qd_d[i] = this->input[i];  // assume inputs are already in m/s, rad/s
      }

      // inputs are already in m/s and rad/s for these modes
      return this->calcFromVels();
      break;

    // ********** IMU/RIDING MODES ***************
    case 1:  // zero IMU
      this->setZeros(input[0], input[1], input[2]);
      break;
    case 2:  // or velocity (2) control. inputs are in IMU lean angle
      // constrain inputs within to +/- pi
      for (int i = 0; i < 3; i++) {
        //this->input[i] = this->dewrap(input[i]);                          // dewrap input to make within ±180°
        //this->input[i] = this->input[i] - this->in0[i];                   // Apply zero offsets to inputs
        //this->input[i] = this->lim(this->input[i]/this->maxLean, -1, 1);  // Normalize inputs to [-1, 1] range
        if (mode == 2) {  // Velocity control
          this->qd_d[i] = this->input[i] * this->qd_max[i];
        } else if (mode == 3) {  // Acceleration control
          this->qdd_d[i] = this->input[i] * this->qdd_max[i];
        }
      }
      if (mode == 3) {
        this->calcFromAccels();  // set velocities based off of acceleration inputs
      }
      return this->calcFromVels();
      break;
    case 3:  // IMU mode, velocity (3)
      return 0;
      break;
    default:
      return 0;
  }
}

int Planner::plan_world(double x_in, double y_in, double z_in, double alpha) {  // For driving the robot wrt world frame. Takes in IMU orientation and transforms inputs to robot frame
  double beta = atan2(y_in, x_in);                                              // angle of desired velocity vector wrt world frame
  double qd_n = sqrt(y_in * y_in + x_in * x_in);                                // velocity magnitude
  double gamma = alpha + beta;                                                  // angle of desired velocity vector wrt robot frame
  return this->plan(qd_n * cos(gamma), qd_n * sin(gamma), z_in);
}

// double bound(double value, double a, double b) {
//   return a < b ? constrain(value, a, b) : constrain(value, b, a);
// }

double min(double a, double b) {
  return a < b ? a : b;
}

int Planner::calcFromVels() {
  double del_qd = 0;  // termporary variable for velocity error
  for (int i = 0; i < 3; i++) {
    // set desired to zero if within deadband
    // if (std::abs(this->qd_d[i]) < this->dband[i]) {
    //   this->qd_d[i] = 0;
    // }

    del_qd = this->qd_d[i] - this->qd[i];  // velocity error term
    //this->del_qd_max[i] = this->qdd_max[i] * this->dt;
    this->qd[i] = this->qd[i] + this->sign(del_qd) * min(std::abs(del_qd), this->del_qd_max[i]);
    this->qd[i] = this->lim(this->qd[i], -1.0 * this->qd_max[i], this->qd_max[i]);
  }
  return 1;
}

int Planner::calcFromAccels() {  // use qdd_d as control input
  double del_qd = 0;
  for (int i = 0; i < 3; i++) {
    del_qd = this->qdd_d[i] * this->dt;
    this->qd_d[i] = this->qd_d[i] + del_qd;
    this->qd_d[i] = this->lim(this->qd_d[i], -this->qd_max[i], this->qd_max[i]);
  }
  return 1;
}

// Getters, Setters
void Planner::setMode(int mode) {
  this->mode = mode;
}

void Planner::setZeros(double x_in, double y_in, double z_in) {
  this->input[0] = x_in;
  this->input[1] = y_in;
  this->input[2] = z_in;
  for (int i = 0; i < 3; i++) {
    while (std::abs(this->input[i]) > M_PI) {
      if (this->input[i] > M_PI) {
        this->input[i] = this->input[i] - 2 * M_PI;
      }
      if (this->input[i] < -M_PI) {
        this->input[i] = this->input[i] + 2 * M_PI;
      }
    }
    this->in0[i] = input[i];
  }
}

double Planner::getTargetVX() {
  return this->qd[0];
}

double Planner::getTargetVY() {
  return this->qd[1];
}

double Planner::getTargetVZ() {
  return this->qd[2];
}

double Planner::sign(double x) {
  //return (x > 0) - (x < 0);
  if (x > 0) return 1.0;
  if (x < 0) return -1.0;
  return 1.0;  // temporarily setting to 1 for debugging
}

double Planner::lim(double x, double a, double b) {
  if (x < a) {
    return a;
  } else if (b < x) {
    return b;
  } else
    return x;
}

double Planner::dewrap(double x) {  // unwrap angles to make within ±PI (±180 degrees). units are radians
  double ret = x;
  while (std::abs(ret) > M_PI) {
    if (x > M_PI) {
      ret = ret - 2 * M_PI;
    }
    if (ret < -M_PI) {
      ret = ret + 2 * M_PI;
    }
    return ret;
  }
}
