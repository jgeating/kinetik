#include "Planner2.h"
#include <cmath>

/*
*/

Planner::Planner(double tInner, SwerveTrajectory traj, pad_vars padVars, SwerveKinematics kin)
{
  this->dt = tInner / 1000000.0; // length of time step, microseconds

  // Trajectory constaints
  this->maxVelocity[0] = traj.qd_max[0]; // max velocity allowed, m/s or rad/s
  this->maxVelocity[1] = traj.qd_max[1];
  this->maxVelocity[2] = traj.qd_max[2];
  this->dband_teleop[0] = traj.dzt[0]; // deadband for tele-op
  this->dband_teleop[1] = traj.dzt[1];
  this->dband_teleop[2] = traj.dzt[2];

  // Pad steering related
  padx_pid = new PID(padVars.kp[0], padVars.ki[0], padVars.kd[0], dt, padVars.lag[0]);
  pady_pid = new PID(padVars.kp[1], padVars.ki[1], padVars.kd[1], dt, padVars.lag[1]);
  padz_pid = new PID(padVars.kp[2], padVars.ki[2], padVars.kd[2], dt, padVars.lag[2]);
  padx_pid->setSetpoint(0); // setpoint = 0 means try to put human center of pressure at middle of footpad
  pady_pid->setSetpoint(0);
  padz_pid->setSetpoint(0);
  this->maxPadVelocity[0] = padVars.qd_max[0]; // Max velocity allowed during pad riding
  this->maxPadVelocity[1] = padVars.qd_max[1];
  this->maxPadVelocity[2] = padVars.qd_max[2];
  this->maxPadAcceleration[0] = padVars.qdd_max[0]; // Max acceleration allowed during pad riding
  this->maxPadAcceleration[1] = padVars.qdd_max[1];
  this->maxPadAcceleration[2] = padVars.qdd_max[2];

  // Steering variables
  this->steeringGearRatio = kin.rmd_ratio * kin.steering_pulley_ratio;
  this->maxSteeringVelocity = kin.vRobot * kin.kv_steer * RPM2RAD_SEC / this->steeringGearRatio;
  this->maxSteeringAcceleration = this->maxSteeringVelocity * 10; // For now, hardcode to accelerate to full speed in 1/const seconds
  this->maxSteeringError = traj.s_error_max;

  // Kinematics setup
  for (int i = 0; i < 4; i++)
  {
    this->kinematics[i] = new Kinematics(0.25, 0, i); // (radius to wheels, deadband [0 to 1], wheel index)
  }
}

int Planner::plan_teleop(double x_in, double y_in, double z_in, double gain_in)
{
  double input[3] = { x_in, y_in, z_in };
  for (int i = 0; i < 3; i++)
  {

    // Apply deadband
    this->desiredVelocity[i] = abs(input[i]) - this->dband_teleop[i];
    if (this->desiredVelocity[i] < 0.0)
    {
      this->desiredVelocity[i] = 0.0;
    }
    this->desiredVelocity[i] = this->desiredVelocity[i] * sign(input[i]); // scale from 0-1 input to max velocity in m/s

    // Apply global gain (gain_in is from 0 to 1)
    if (i < 2)
    {
      this->desiredVelocity[i] = this->desiredVelocity[i] * gain_in; // Scale up to max velocity using left stick vertical (throttle, no spring center)
    }
    else
    {
      this->desiredVelocity[i] = -this->desiredVelocity[i] * (gain_in + 1.0) / 2.0; // for steer, only scale between 50% and 100%, not 0 and 100%
    }

    // Scale from unitless to real values
    this->desiredVelocity[i] = this->desiredVelocity[i] * this->maxVelocity[i];
  }

  // inputs are already in m/s and rad/s for these modes
  this->calcFromVels(); // Apply robot level slewing limits, etc.
  // this->regularize();
  return 0;
}

enum class PadControlMode
{
  DEACTIVATED,
  VELOCITY,
  ACCELERATION,
};

enum PadAxis
{
  PAD_X = 0,
  PAD_Y = 1,
  PAD_Z = 2,
};

int Planner::plan_pads(double x_in, double y_in, double z_in, double gain_in)
{
  padx_pid->setInput(x_in);
  pady_pid->setInput(y_in);
  padz_pid->setInput(z_in);

  PadControlMode control_mode[] = {PadControlMode::DEACTIVATED, PadControlMode::DEACTIVATED, PadControlMode::VELOCITY}; // sets control mode of each axis hardcoded for now. 0 = deactivated, 1 = velocity mode, 2 = acceleration control

  this->setZeros(input[0], input[1], input[2], gain_in);
  for (int i = 0; i < 3; i++)
  {
    double pid_output = 0;
    switch (i)
    {
    case PAD_X:
      pid_output = padx_pid->compute();
      break;
    case PAD_Y:
      pid_output = pady_pid->compute();
      break;
    case PAD_Z:
      pid_output = padz_pid->compute();
      break;
    }
    switch (control_mode[i])
    {
    case PadControlMode::VELOCITY:
      this->desiredVelocity[i] = constrain(pid_output, -this->maxPadVelocity[i], this->maxPadVelocity[i]);
      break;
    case PadControlMode::ACCELERATION:
      this->desiredAcceleration[i] = constrain(pid_output, -this->maxPadAcceleration[i], this->maxPadAcceleration[i]);
      this->desiredVelocity[i] = constrain(this->desiredVelocity[i] - this->desiredAcceleration[i] * this->dt, -this->maxPadVelocity[i], this->maxPadVelocity[i]);
      break;
    }
  }
  this->calcFromVels();
  // this->regularize();
  return 1;
}

int Planner::plan_world(double x_in, double y_in, double z_in, double gain_in, double alpha)
{                                                // For driving the robot wrt world frame. Takes in IMU orientation and transforms inputs to robot frame
  double beta = atan2(y_in, x_in);               // angle of desired velocity vector wrt world frame
  double qd_n = sqrt(y_in * y_in + x_in * x_in); // velocity magnitude
  double gamma = alpha + beta;                   // angle of desired velocity vector wrt robot frame
  return this->plan_teleop(qd_n * cos(gamma), qd_n * sin(gamma), z_in, gain_in);
}

int Planner::steerTo(double ang, int ind)
{
  this->steeringError[ind] = ang - this->steeringAngle[ind]; // steer error term
  if (abs(this->steeringError[ind]) > PI)
  { // Unwrap
    while (abs(this->steeringError[ind]) > PI)
    {
      this->steeringError[ind] = this->steeringError[ind] - (sign(this->steeringError[ind]) * 2.0 * PI);
    }
  }

  // Determine preferred direction (fwd vs. reverse)
  if (abs(this->steeringError[ind]) > PI / 2.0)
  {
    this->steeringError[ind] = this->steeringError[ind] + (sign(this->steeringError[ind]) * -PI);
    this->steeringDirection[ind] = -1.0;
  }
  else
  {
    this->steeringDirection[ind] = 1.0;
  }

  double tStop = abs(this->steeringVelocity[ind]) / this->maxSteeringAcceleration;                               // minimum time until motor can stop
  double minStop = abs(this->steeringVelocity[ind]) * tStop + this->maxSteeringAcceleration * tStop * tStop / 2; // minimum distance in which motor can stop
  // double acc = constrain(abs(delPos)/rampDist, -1.0, 1.0) * aMax; // To prevent instabilities and numerical error buildup near zero
  // if (sign(delPos*v[mot]) == -1) v[mot] = 0;      // velocity should never be pushing away from setpoint, even if it breaks max accel limit
  // double acc = ((abs(this->s_error[ind]) > minStop) - 0.5) * 2.0;  // takes a bool for acceleration direction and makes it 1 or -1
  if (abs(this->steeringError[ind]) > minStop)
  {
    this->steeringVelocity[ind] = this->steeringVelocity[ind] + sign(this->steeringError[ind]) * this->maxSteeringAcceleration * this->dt; // Speed up or coast (coasting done with constrain)
  }
  else
  {
    if (abs(this->steeringAngle[ind]) < this->maxSteeringVelocity * this->dt)
    {
      this->steeringVelocity[ind] = 0; // If almost at zero, just set to zero. This prevents oscillating about zero with qd_max vel
    }
    else
    {
      this->steeringVelocity[ind] = this->steeringVelocity[ind] - sign(this->steeringVelocity[ind]) * this->maxSteeringAcceleration * this->dt; // Slow down
    }
  }
  this->steeringVelocity[ind] = lim(this->steeringVelocity[ind], -1 * this->maxSteeringVelocity, this->maxSteeringVelocity);

  double delYaw = this->steeringVelocity[ind] * this->dt;
  if (abs(delYaw) > abs(this->steeringError[ind])){
    delYaw = this->steeringError[ind];
  }
  this->steeringAngle[ind] = this->steeringAngle[ind] + delYaw;
  this->steeringPosition[ind] = this->steeringPosition[ind] - delYaw * this->steeringGearRatio; // Minus sign added 1/6/2024 because steering was reversed. definitely a better way
  return 0;
}

int Planner::driveTo(double vel, int ind)
{
  this->driveVelocity[ind] = vel * this->steeringDirection[ind];
  return 0;
}

int Planner::calcFromVels() // Robot level slew limits etc. would be applied here
{
  for (int i = 0; i < 4; i++)
  {
    kinematics[i]->calc(this->desiredVelocity[0], this->desiredVelocity[1], this->desiredVelocity[2]);
    this->steerTo(kinematics[i]->getTargetSteer(), i);
    this->driveTo(kinematics[i]->getTargetVel(), i);
  }
  return 1;
}

int Planner::calcFromAccels()
{ // use qdd_d as control input
  double del_qd = 0;
  for (int i = 0; i < 3; i++)
  {
    del_qd = this->desiredAcceleration[i] * this->dt;
    this->desiredVelocity[i] = this->desiredVelocity[i] + del_qd;
    this->desiredVelocity[i] = this->lim(this->desiredVelocity[i], -this->maxVelocity[i], this->maxVelocity[i]);
  }
  return 1;
}

double min2(double a, double b)
{
  return a < b ? a : b;
}

void Planner::eStop()
{
  for (int i = 0; i < 4; i++)
  {
    this->driveVelocity[i] = 0;
  }
  for (int i = 0; i < 3; i++)
  {
    this->desiredVelocity[i] = 0;
  }
}
double Planner::getTargetVX()
{
  return this->desiredVelocity[0];
}
double Planner::getTargetVY()
{
  return this->desiredVelocity[1];
}
double Planner::getTargetVZ()
{
  return this->desiredVelocity[2];
}
void Planner::setSteerAngle(double ang, int ind)
{
  this->steeringAngle[ind] = ang;
}
void Planner::setMotAngle(double angle, int ind)
{
  this->steeringPosition[ind] = angle;
}
double Planner::getSteerAngle(int ind)
{
  return this->steeringAngle[ind];
}
double Planner::getMotAngle(int ind)
{
  return this->steeringPosition[ind];
}
double Planner::getDriveWheelSpeed(int ind)
{
  return this->driveVelocity[ind];
}

double Planner::sign(double x)
{
  // return (x > 0) - (x < 0);
  if (x > 0)
    return 1.0;
  if (x < 0)
    return -1.0;
  return 1.0; // temporarily setting to 1 for debugging
}
double Planner::lim(double x, double a, double b)
{
  if (x < a)
  {
    return a;
  }
  else if (b < x)
  {
    return b;
  }
  else
    return x;
}
