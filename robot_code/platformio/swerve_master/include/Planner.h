#ifndef _PLANNER_
#define _PLANNER_

#include "Kinematics.h"
#include "Swerve.h"
#include "PID.h"
#include "state.h"
#include "shared/utils.h"

// Robot level planning
class Planner
{
private:
  // Generic
  double rpm2rad_sec = 2 * PI / 60;
  double temp = 0; // Temp variable for debugging

  // safety, timing, state related
  int mode;               // mode for what inputs/control algorithm to use
  double dt;              // length of time step, microseconds
  double dband_teleop[3]; // Deadband of controller when in teleop mode. In fraction of total command (0 to 1)

  // Vest related
  double in0[3] = {0, 0, 0};   // zeros for input angles from IMU(s), rad
  double input[3] = {0, 0, 0}; // input for various control modes, units dependent on control mode

  // Pads related
  PID *padx_pid;
  PID *pady_pid;
  PID *padz_pid;
  double p_qd_max[3];
  double p_qdd_max[3];

  // Kinematics/trajectory related
  double q[3] = {0, 0, 0};     // Current position/orientation, x/y/yaw, m or rad
  double qd[3] = {0, 0, 0};    // Current velocity, m/s or rad/s
  double qdd[3] = {0, 0, 0};   // current acceleration, m/s^2 or rad/s^2
  double q_d[3] = {0, 0, 0};   // desired position, m or rad
  double qd_d[3] = {0, 0, 0};  // desired velocity, m/s or rad/s
  double qdd_d[3] = {0, 0, 0}; // Desired acceleration, m/s^2 or rad/s^2
  double qd_max[3];            // max velocity allowed, m/s or rad/s
  double qdd_max[3];           // max acceleration allowed, m/s^2 or rad/s^2
  double dband[3];             // dead band zone, in m/s or rad/s. robot will not move if desired is slower than this
  double del_qd_max[3];        // intermediate variable for calculations
  double alpha = 0;            // angle of robot wrt world frame, rad
  double gain = 0;             // Gain multiplier for slow driving. 0 to 1
  double qd_r[3];              // Regularization vector
  double vv = 0;               // velocity vector
  double vvd = 0;              // derivative of velocity vector

  // Steering variables
  double s_q[4] = {0, 0, 0, 0};     // Current steering angle of each wheel, radians
  double s_qd[4] = {0, 0, 0, 0};    // Current steering velocity of each wheel, rad/s
  double s_mot_q[4] = {0, 0, 0, 0}; // Current steering motor positions, radians
  double s_state[4] = {0, 0, 0, 0}; // Current steering state
  double s_error[4] = {0, 0, 0, 0}; // Steering error, radians
  double s_dir[4] = {1, 1, 1, 1};   // Contains direction of wheel, whether inverted (1) or not (-1)
  double s_ratio;                   // Steering gear ratio, motor to yaw stage
  double s_qd_max;                  // max steering stage velocity, rad/s
  double s_qdd_max;                 // max steering stage acceleration, rad/s^2
  double s_error_max;
  int min_tracking_wheels;

  // Drive variables
  double d_qd[4] = {0, 0, 0, 0};
  double d_qdd[4] = {0, 0, 0, 0};
  double d_qd_max;
  double d_qdd_max;

  // Kinematics object for robot to wheel frame calcs
  Kinematics **kinematics = new Kinematics *[4];

  // State object for tracking various robot state estimates, etc.
  state *bot_state;

  // Functions
  int steerTo(double ang, int ind); // Steers wheels based on slew and state limits
  int driveTo(double vel, int ind); // Adjusts wheel velocity state vs. desired based on slew limits etc.
  int calcFromAccels();             // for planning in acceleration control
  int calcFromVels();               // main private planning function
  int regularize();
  double sign(double x);                    // returns the sign of the number, either 1 or -1
  double lim(double x, double a, double b); // arduino contrain port, for doubles

public:
  Planner(double tInner, SwerveTrajectory traj, pad_vars padVars, SwerveKinematics kin);

  // Main functions
  int plan_teleop(double x_in, double y_in, double z_in, double gain_in);
  int plan_pads(double x_in, double y_in, double z_in, double gain_in);
  int plan_world(double x_in, double y_in, double z_in, double gain_in, double alpha);

  // Setters, getters
  void eStop();
  void setMode(int mode);
  void setSteerAngle(double ang, int ind);
  void setMotAngle(double angle, int ind);
  void setZeros(double x_in, double y_in, double z_in, double gain_in);
  double getTargetVX();
  double getTargetVY();
  double getTargetVZ();
  double get_vv();
  double get_vvd();
  double getSteerAngle(int ind);
  double getMotAngle(int ind);
  double getDriveWheelSpeed(int ind);
  double getTemp(); // temp function for debugging private vars
};

#endif
