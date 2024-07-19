#ifndef _PLANNER_
#define _PLANNER_

#include "Kinematics.h"
#include "Swerve.h"
#include "PID.h"
#include "state.h"
#include "shared/utils.h"

#define RPM2RAD_SEC = 2 * PI / 60

// Robot level planning
class Planner
{
private:
  // safety, timing, state related
  double dt;              // length of time step, microseconds
  double dband_teleop[3]; // Deadband of controller when in teleop mode. In fraction of total command (0 to 1)

  // Pads related
  PID *padx_pid;
  PID *pady_pid;
  PID *padz_pid;
  double maxPadVelocity[3];
  double maxPadAcceleration[3];

  // Kinematics/trajectory related
  double desiredVelocity[3] = {0, 0, 0};  // desired velocity, m/s or rad/s
  double desiredAcceleration[3] = {0, 0, 0}; // Desired acceleration, m/s^2 or rad/s^2
  double maxVelocity[3];            // max velocity allowed, m/s or rad/s

  // Steering variables
  double steeringAngle[4] = {0, 0, 0, 0};     // Current steering angle of each wheel, radians
  double steeringVelocity[4] = {0, 0, 0, 0};    // Current steering velocity of each wheel, rad/s
  double steeringPosition[4] = {0, 0, 0, 0}; // Current steering motor positions, radians
  double steeringError[4] = {0, 0, 0, 0}; // Steering error, radians
  double steeringDirection[4] = {1, 1, 1, 1};   // Contains direction of wheel, whether inverted (1) or not (-1)
  double steeringGearRatio;                   // Steering gear ratio, motor to yaw stage
  double maxSteeringVelocity;                  // max steering stage velocity, rad/s
  double maxSteeringAcceleration;                 // max steering stage acceleration, rad/s^2
  double maxSteeringError;

  // Drive variables
  double driveVelocity[4] = {0, 0, 0, 0};

  // Kinematics object for robot to wheel frame calcs
  Kinematics **kinematics = new Kinematics *[4];

  // Functions
  int steerTo(double ang, int ind); // Steers wheels based on slew and state limits
  int driveTo(double vel, int ind); // Adjusts wheel velocity state vs. desired based on slew limits etc.
  int calcFromAccels();             // for planning in acceleration control
  int calcFromVels();               // main private planning function
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
  void setSteerAngle(double ang, int ind);
  void setMotAngle(double angle, int ind);
  double getTargetVX();
  double getTargetVY();
  double getTargetVZ();
  double getSteerAngle(int ind);
  double getMotAngle(int ind);
  double getDriveWheelSpeed(int ind);
};

#endif
