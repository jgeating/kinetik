#ifndef _SWERVE_
#define _SWERVE_

#include "Arduino.h";
#include "Kinematics.h"; // wheel level kinematics/trigonometry
#include "shared/utils.h";           // Basic utils like more powerful serial
#include "shared/Channel.h";         // for RC PWM inputs

#define YAW_GEAR_RATIO -18 // RMD-X6 planetary ratio = 8:1, pulley ratio = 72/32 = 2.25

struct pad_vars
{
  // float kp_x = 30;
  // float kp_y = 20;
  // float kp_z = 15;

  float kp_x = 80;
  float kp_y = 20;
  float kp_z = 50;

  float ki_x = 0;
  float ki_y = 0;
  float ki_z = 0;

  // should always be negative
  float kd_x = -8;
  float kd_y = 0;
  float kd_z = 0;

  float lag_x = 10; // number of lag samples
  float lag_y = 10;
  float lag_z = 10;
};

struct vest_vars
{
  double kp_x = 25;           // proportional gain for lean controller (forwards/backwards). Multiples into (m/s^2) / rad
  double kp_y = 100;           // proportional gain for lean controller (left/right). Multiples into (m/s^2) / rad
  double kp_z = 15;           // proportional gain for lean controller (rotation). Multiples into (rad/sec^2) / rad
  double kd_x = .05;             // derivative gain for lean controller. Multiples into (m/s^2) / (rad/sec) 
  double kd_y = .3;          // derivative gain for lean controller. Multiples into (m/s^2) / (rad/sec) 
  double kd_z = 0;             // derivative gain for lean controller. Multiples into (rad/s^2) / (rad/sec) 

  double x = 0;              // x angle of imu vest
  double y = 0;              // y angle of imu vest
  double z = 0;              // z angle of imu vest
  double maxLean = M_PI / 4; // max lean angle, to scale to output
  double xZero = 0;          // Zero angle of vest. Set when calibrating starting lean angle
  double yZero = 0;
  double zZero = 0;
  double xRate = 0;          // x rotation rate (direct gyro signal)
  double yRate = 0;
  double zRate = 0;
};

struct imu_vars
{
    // Robot stuff
  double zRobot = 0;        // imu yaw angle of robot 
  double zRobotRate = 0;    // yaw rate of robot (direct gyro signal)
  double zZero_robot = 0;   // zero yaw angle of robot

};

// Robot level trajectory/control
struct SwerveTrajectory
{
  double qd_max[3] = {40, 40, 40};  // max velocity: {m/s, m/s, rad/s}
  double qdd_max[3] = {60, 60, 75}; // max acceleration: {m/s^2, m/s^2, rad/s^2}
  double dz[3] = {.1, .1, .1};      // Deadzone velocity bounds: {m/s, m/s, rad/s}
  double qd_d[3] = {0, 0, 0};       // desired velocity, to send to planner
  double qdd_d[3] = {0, 0, 0};      // desired acceleration
  double input[3] = {0, 0, 0};      // hold inputs
};

struct SwerveKinematics
{
  // Kinematics
  int nWheels = 4;
  int pole_pairs = 7; // number of pole pairs in hub motors. Assuming 7 pole pairs from research. did not actually measure. Used for converting erpm to rpm, to calculate real velocities.
  int dRatio;         // used to convert target wheel rad/sec to erpm
  Kinematics **kinematics = new Kinematics *[nWheels];
  bool calibrated = 0; // Whether or not the robot has been calibrated
};

struct LoopTiming
{
  // Loop Timing Variables
  double tInner = 4000;                                 // Target length of inner loop controller, microseconds
  double tOuter = 50000;                                // Target length of outer (second) loop controller, microseconds
  unsigned long lastInner = 0;                          // last time counter of inner loop
  unsigned long lastOuter = 0;                          // last time counter of outer loop
  unsigned long now = 0;                                // variable to store current time, to avoid multiple calls to micros() function
  bool behind = 0;                                      // If Due can't keep up with loop rate, this bool will become true
  unsigned long timer[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // Timers used to measure computation lengths of various functions, loops, etc.
};

struct SwerveCAN
{
  byte cTxData0[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int idd = 1 || CAN_PACKET_SET_RPM << 8; // Drive ID
  bool ext = true;
  int len = 4;
  uint32_t pos = 0;
  // For receiving
  short rxData = 0;
  long lMsgID;
  bool bExtendedFormat;
  byte cRxData[8];
  byte cDataLen;
  int driveCanDelay = 100; // # of microseconds to delay sending drive CAN for
  // CAN stuff for steering motors
  byte cTxData1[8] = {0xA3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int idy = 1 || CAN_PACKET_SET_POS << 8; // Yaw ID
  unsigned char buf[8];
  int32_t ang = 0;
  int steerCanDelay = 100; // # of microseconds to delay sending steering CAN for
};

// PWM/Receiver stuff
struct PWMReceiver
{
  short chs = 8;                                                     // number of channels to read from receiver
  short chOff[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; // Channel offsets (calibrate to find these)
  Channel **channels = new Channel *[chs];
  int rcTimeout = 100000; // number of microseconds before receiver timeout is tripped - make sure it is a good bit longer than 2000 microseconds
  bool rcLost = 1;        // This is set to true if no receiver signal is received within the rcTimeout timeframe (microseconds)
  int estop_ch = 6;       // which Rx channel is used for motor enabling/SW e-stop. 0 index
  int mode_ch = 7;        // which Rx channel is used for setting mode
};

// Modes, safety, e-stop, debug
struct Modes
{
  char buff[100];       // String buffer for Serial
  int mode = 0;         // 0 = RC mode (teleop), 1 = weight control mode
  int eStop = 0;        // e-stop variable for safety
  bool debugRx = 0;     // whether or not to debug receiver
  bool debugTiming = 1; // whether or not to debug timing, look at loop lengths, etc.
  bool debugRiding = 0;
  bool zeroing = 0;     // whether or not system is zeroing 
};

// Robot state stuff
struct RobotState
{
  int ir[4] = {0, 0, 0, 0}; // status of IR sensor
  // looking from bottom, (+) to offset rotates wheel CCW
  double irPos[4] = {90-19.2, 270-9.5, 270-19.2, 90-9.6}; // absolute position if IR sensors, for calibrating position on startup, degrees. increasing rotates clockwise looking from the top
  int irPin[4] = {22, 24, 26, 28};                              // pins that ir sensors are hooked up to
  double mRPM[4] = {0, 0, 0, 0};                                // Speed of drive motors (pre gear stage). In eRPM, I think...
  double yRatio = YAW_GEAR_RATIO;                               // Yaw pulley stage ratio, >1
  int motPol[4] = {1, 1, 1, 1};                                 // Used to switch motor direction depending on VESC configuration. Not implemented yet due to datatype issues. Just changing VESC parameters instead
};

struct Watchdog
{
  unsigned long loopTimeMicros = 4500;
  unsigned long timeBetweenReportsMicros = 1000000;
  unsigned long prevReportTime = 0;
  unsigned long prevLoopTime = 0;
};

void printWatchdogError(Watchdog &watchdog);
void serialPrintln(int bufferSize, const char *format, ...);
void serialPrint(int bufferSize, const char *format, ...);

#endif
