// HAL includes for cross-platform compatibility
#include "hal/HALConfig.h"

#if HAL_IMPLEMENTATION == HAL_SIM
#include "hal/HALFactory.h"
#include "hal/ArduinoCompat.h"
#endif

#include <math.h>             // Math functions

#if HAL_IMPLEMENTATION == HAL_REAL
#include "Kinematics.h"       // wheel level kinematics/trigonometry
#include "Planner.h"          // robot level planning
#include "shared/utils.h"     // Basic utils like more powerful serial
#include "penny/Steer.h"      // For controlling steering actuator
#include "penny/Pads.h"       // For interfacing with weight pads
#include "penny/Drive.h"      // For controlling drive motors
#include <Wire.h>             // For accessing native Arduino I2C functions
#include "Adafruit_Sensor.h"  // Downloaded library for IMU stuff
#include "Adafruit_BNO055.h"  // Downloaded library for IMU stuff
#include "utility/imumaths.h" // Downloaded library for IMU stuff
#include "SbusReceiver.h"
#include "Swerve.h"
#include "Performance.h"
#include "PID.h"                    // For PID controllers
#include "shared/LowPassFilter.cpp" // Low pass filter class
#include "penny/Lights.h"
#include "SwerveTelemetry.h"
#endif

// Definitions
#pragma region

#define RADIUS_SWERVE_ASSEMBLY 0.25 // distance to wheel swerve axes, meters
#define DEAD_ZONE 0.1
#define pi 3.14159265358979
#define TELEMETRY_REPORT_PERIOD 500000
#define MCU "DUE" // Either "T4_1", or "DUE"

#if HAL_IMPLEMENTATION == HAL_REAL
// Instantiate structs
SwerveTrajectory traj;
pad_vars padVars;
SwerveKinematics kin;
LoopTiming loopTiming;
Profiles profiles;
Modes modes; // Handles different driving and control modes

// Instantiate classes
Planner *planner; // for planning robot level motion
Pads *pads;       // For driving with force pads
PID *padx_pid;    // PID controller for weight (pad) steering in X (sideways) axis
PID *pady_pid;    // PID controller for weight (pad) steering in Y (forwards) axis
PID *padz_pid;    // PID controller for weight (pad) steering in Z (rotation) axis
Lights *lights;   // Controls LED strips for signals/entertainment

SwerveCAN can;
Drive::Type types[] = {Drive::Type::ODRIVE, Drive::Type::ODRIVE, Drive::Type::ODRIVE, Drive::Type::ODRIVE};

RobotState robotState;
Drive **drive = new Drive *[kin.nWheels];
Steer **steer = new Steer *[kin.nWheels];
double aMaxSteer = 5000;  // Max angular acceleration of steer motor, in motor frame, rad/s^2. Safe starting value: 5000
double wMaxSteer = 10000; // Max angular velocity of steer motor, in motor frame, rad/s. Safe starting value: 10000
int doneHoming = 0;       // Used to determine when calibration sequence is finished. 1 = finished.

// Plotting, telemetry
int plotCounter = 0;
unsigned long prevTelemetryReportTime = 0;
Watchdog watchdog;
LowPassFilter filter(10); // create a low-pass filter with 10 readings

SwerveTelemetry swerveTelemetry;
SbusReceiver sbusReceiver;
#else
// Simulation mode - minimal variables
unsigned long prevTelemetryReportTime = 0;

// Forward declarations for simulation functions
void calMotor();
#endif

#pragma endregion

// Helper functions to abstract RC receiver calls for cross-platform compatibility
inline void rcReceiver_read() {
#if HAL_IMPLEMENTATION == HAL_REAL
  sbusReceiver.read();
#else
  // In simulation, HAL RC receiver is updated automatically
  if (HAL::rcReceiver) {
    HAL::rcReceiver->update();
  }
#endif
}

inline double rcReceiver_getRightHor() {
#if HAL_IMPLEMENTATION == HAL_REAL
  return sbusReceiver.getRightHor();
#else
  return HAL::rcReceiver ? HAL::rcReceiver->getRightHorizontal() : 0.0;
#endif
}

inline double rcReceiver_getRightVert() {
#if HAL_IMPLEMENTATION == HAL_REAL
  return sbusReceiver.getRightVert();
#else
  return HAL::rcReceiver ? HAL::rcReceiver->getRightVertical() : 0.0;
#endif
}

inline double rcReceiver_getLeftHor() {
#if HAL_IMPLEMENTATION == HAL_REAL
  return sbusReceiver.getLeftHor();
#else
  return HAL::rcReceiver ? HAL::rcReceiver->getLeftHorizontal() : 0.0;
#endif
}

inline double rcReceiver_getLeftVert() {
#if HAL_IMPLEMENTATION == HAL_REAL
  return sbusReceiver.getLeftVert();
#else
  return HAL::rcReceiver ? HAL::rcReceiver->getLeftVertical() : 0.0;
#endif
}

inline double rcReceiver_getHandheld() {
#if HAL_IMPLEMENTATION == HAL_REAL
  return sbusReceiver.getHandheld();
#else
  return HAL::rcReceiver ? HAL::rcReceiver->getLeftKnob() : 0.0;
#endif
}

inline double rcReceiver_getRightKnob() {
#if HAL_IMPLEMENTATION == HAL_REAL
  return sbusReceiver.getRightKnob();
#else
  return HAL::rcReceiver ? HAL::rcReceiver->getRightKnob() : 0.0;
#endif
}

inline bool rcReceiver_isBlueSwitchUp() {
#if HAL_IMPLEMENTATION == HAL_REAL
  return sbusReceiver.isBlueSwitchUp();
#else
  return HAL::rcReceiver ? HAL::rcReceiver->isBlueSwitchUp() : false;
#endif
}

inline bool rcReceiver_isBlueSwitchDown() {
#if HAL_IMPLEMENTATION == HAL_REAL
  return sbusReceiver.isBlueSwitchDown();
#else
  return HAL::rcReceiver ? HAL::rcReceiver->isBlueSwitchDown() : false;
#endif
}

inline bool rcReceiver_isBlueSwitchCentered() {
#if HAL_IMPLEMENTATION == HAL_REAL
  return sbusReceiver.isBlueSwitchCentered();
#else
  return HAL::rcReceiver ? HAL::rcReceiver->isBlueSwitchCentered() : true;
#endif
}

inline double rcReceiver_getRedSwitch() {
#if HAL_IMPLEMENTATION == HAL_REAL
  return sbusReceiver.getRedSwitch();
#else
  return HAL::rcReceiver ? HAL::rcReceiver->getRedSwitch() : 1.0;
#endif
}

inline bool rcReceiver_rcLost() {
#if HAL_IMPLEMENTATION == HAL_REAL
  return sbusReceiver.rcLost();
#else
  return HAL::rcReceiver ? HAL::rcReceiver->isSignalLost() : false;
#endif
}

void setup()
{
#if HAL_IMPLEMENTATION == HAL_SIM
  // Initialize HAL system for simulation
  HAL::initialize();

  // Simulation setup
  Serial.begin(460800);
  Serial.println("Simulation setup complete!");

#else
  // Real hardware setup
  // Serial and CAN setup
  Serial.begin(460800); // Bumping up serial rate 7/21/2024 for serial telemetry over usb to computer
  delay(400);

  analogReadResolution(12);

  motors::canBus1.begin();
  motors::canBus1.setBaudRate(1000000);

  delay(500);

  for (int i = 0; i < 4; i++) {
    motors::steer[i].printMessage();
  }

  sbusReceiver.init();

  for (int i = 0; i < 4; i++)
  {
    // This is causing CAN stuff on the steering motors not to work for some reason
    // pinMode(robotState.irPin[i], INPUT);
  }

  // Set up digital I/O
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW); // Set up indicator LED

  Serial.println("CAN and Pins initialized. Setting up kinematics and trajectory planning objects...");

  delay(3000); // Need a delay between CAN initialization and ODRIVE initialization (in the Drive class)

  // Kinematics and path planning setup
  kin.dRatio = kin.pole_pairs * 60 / (2 * M_PI) / (.083 / 2); // used to convert m/s to rpm
  for (int i = 0; i < kin.nWheels; i++)
  {
    robotState.irPos[i] = robotState.irPos[i]; // Correcting for polar coordinate frame
    steer[i] = new Steer(wMaxSteer, aMaxSteer, kin.yRatio, loopTiming.tInner, can.len, i);
    drive[i] = new Drive(traj.qd_max[0], traj.qdd_max[0], kin.dRatio, loopTiming.tInner, can.len, i, types[i]);
    kin.kinematics[i] = new Kinematics(RADIUS_SWERVE_ASSEMBLY, DEAD_ZONE, i);
  }

  // Set up pad pid classes
  double dt = loopTiming.tInner / 1000000.0;
  pads = new Pads();
  padx_pid = new PID(padVars.kp[0], padVars.ki[0], padVars.kd[0], dt, padVars.lag[0]);
  pady_pid = new PID(padVars.kp[1], padVars.ki[1], padVars.kd[1], dt, padVars.lag[1]);
  padz_pid = new PID(padVars.kp[2], padVars.ki[2], padVars.kd[2], dt, padVars.lag[2]);
  // lights = new Lights(2, 65);

  padx_pid->setSetpoint(0); // setpoint = 0 means try to put human center of pressure at middle of footpad
  pady_pid->setSetpoint(0);
  padz_pid->setSetpoint(0);

  planner = new Planner(loopTiming.tInner, traj, padVars, kin);

  pads->calibrate(); // Zero footpads, assuming zero weight on them

  // swerveTelemetry.start();

  Serial.println("Startup Complete.");
  delay(1000);
#endif
}

void teleop()
{
#if HAL_IMPLEMENTATION == HAL_REAL
  for (int k = 0; k < 4; k++)
  {
    if (k == 0) {
      traj.input[k] = constrain(rcReceiver_getRightHor(), -1.0, 1.0);
    } else if (k == 1) {
      traj.input[k] = constrain(rcReceiver_getRightVert(), -1.0, 1.0);
    } else if (k == 2) {
      traj.input[k] = constrain(rcReceiver_getLeftHor(), -1.0, 1.0);
    } else {
      traj.input[k] = constrain(rcReceiver_getLeftVert() * .5, -0.5, 0.5) + 0.5;
    }
  }
  planner->plan_teleop(traj.input[0], traj.input[1], traj.input[2], traj.input[3]);
#else
  // Simulation mode - just print RC values
  Serial.print("RC: RH=");
  Serial.print(rcReceiver_getRightHor());
  Serial.print(" RV=");
  Serial.print(rcReceiver_getRightVert());
  Serial.print(" LH=");
  Serial.print(rcReceiver_getLeftHor());
  Serial.print(" LV=");
  Serial.println(rcReceiver_getLeftVert());
#endif
}

void padRiding()
{
#if HAL_IMPLEMENTATION == HAL_REAL
  double hand_remote_val = constrain(rcReceiver_getHandheld() * .5, -0.5, 0.5) + 0.5; // ch 3 rewired to read value from handheld e-skate remote
  bool hand_remote_estopped = hand_remote_val < 0.2 && hand_remote_val > -0.2;
  bool hand_zeroing = hand_remote_val <= -0.2;
  bool hand_active_driving = hand_remote_val > 0.2;
  pads->calcVector();

  double x = pads->getX();
  double y = pads->getY();
  double z = pads->getZ();

  planner->plan_pads(x, y, z, hand_remote_val);
  if (rcReceiver_isBlueSwitchDown() || modes.zeroing || hand_zeroing || hand_remote_estopped)
  {
    planner->eStop();
  }
#else
  // Simulation mode - just print handheld value
  Serial.print("Handheld: ");
  Serial.println(rcReceiver_getHandheld());
#endif
}

void updateLoopTiming()
{
#if HAL_IMPLEMENTATION == HAL_REAL
  if (loopTiming.behind)
  { // This means we can't keep up with the desired loop rate. Trip LED to indicate so
    digitalWrite(13, HIGH);
    loopTiming.lastInner = loopTiming.now;
  }
  else
  {
    digitalWrite(13, LOW);
    loopTiming.lastInner = loopTiming.lastInner + loopTiming.tInner;
    loopTiming.behind = true;
  }
#endif
}

void zeroFootPads()
{
#if HAL_IMPLEMENTATION == HAL_REAL
  if (!modes.zeroing)
  {
    Serial.println("Zeroing entered");
    modes.zeroing = true;
  }
  switch (modes.mode)
  {
  case Mode::WEIGHT_CONTROL:
    Serial.println("IMU zeroing not supported");
    break;
  case Mode::PADS:
    // Serial.println("Pads zeroing");
    pads->calibrate();
    planner->eStop();
    break;
  default:
    Serial.println("Unable to parse mode to enter zeroing. This is probably a bug");
  }
#else
  Serial.println("Zeroing not supported in simulation");
#endif
}

void loop()
{
#if HAL_IMPLEMENTATION == HAL_SIM
  // Simulation loop - minimal functionality
  static int loopCount = 0;
  loopCount++;

  if (loopCount % 1000 == 0) {
    Serial.println("Simulation loop running...");
  }

  // Read RC receiver
  rcReceiver_read();

  // Simple telemetry
  if (micros() - prevTelemetryReportTime > TELEMETRY_REPORT_PERIOD) {
    Serial.print("Loop count: ");
    Serial.println(loopCount);

    // Test RC receiver values
    if (rcReceiver_getRightKnob() > 0.6) {
      calMotor(); // Simulation version
    }

    prevTelemetryReportTime = micros();
  }

#else
  // Real hardware loop
  // startProfile(profiles.robotLoop);
  // printWatchdogError(watchdog);
  rcReceiver_read();
  telemetry();

  loopTiming.now = micros();

  if (loopTiming.now - loopTiming.lastInner > loopTiming.tInner)
  {
    updateLoopTiming();

    switch (modes.mode)
    { // Primary case statement for handling modes
    case Mode::TELEOP:
      teleop();
      break;
    case Mode::PADS:
      padRiding();
      break;
    }
    for (int i = 0; i < kin.nWheels; i++)
    {
      steer[i]->motTo(planner->getMotAngle(i), planner->getMotSteerVel(i), rcReceiver_getRedSwitch(), rcReceiver_rcLost()); // Red, (-) is up
      delayMicroseconds(can.steerCanDelay);                                                     // Nasty bug where going from 3 motors to 4 per bus required a 100 us delay instead of 50
      drive[i]->setVel(-planner->getDriveWheelSpeed(i), rcReceiver_getRedSwitch(), rcReceiver_rcLost());
      delayMicroseconds(can.driveCanDelay);
    }
  }
  else // Overtiming
  {
    loopTiming.behind = false;
  }

  if (loopTiming.now - loopTiming.lastOuter > loopTiming.tOuter) // Slow loop
  {
    loopTiming.lastOuter = loopTiming.lastOuter + loopTiming.tOuter;
    //***********************BEGIN SLOW LOOP*******************************

    // Check channels, modes
    if (rcReceiver_getRightKnob() > .6)
    {
      calMotor(can); // Calibrate motors
    }

    if (rcReceiver_isBlueSwitchUp())
    { // default mode is tele-op, blue stick top position
      if (modes.mode != Mode::TELEOP)
      {
        Serial.println("Mode 0 entered (tele-op)");
      }
      modes.mode = Mode::TELEOP;
      planner->setMode(0);
      modes.eStop = false; // Also disable e-stop if tripped
      modes.zeroing = false;
    }

    if (rcReceiver_isBlueSwitchCentered()) // Mode switch is centered
    {                                       // IMU zeroing mode
      if (modes.mode != Mode::PADS)
      {
        Serial.println("Mode 2 entered (pad steering)");
        modes.mode = Mode::PADS;
        delay(100);
      }
      modes.zeroing = false;
    }

    // remote pulled back or transmitter in zeroing mode
    if (rcReceiver_getHandheld() < -.1 || rcReceiver_isBlueSwitchDown())
    {
      zeroFootPads();
    }

    if (rcReceiver_getRedSwitch() < .9 || rcReceiver_rcLost())
    { // Safety loop. This runs if motors aren't meant to be spinning
      // Serial.println("Shutting off ODrive Motor ID 1");
      for (int i = 0; i < 4; i++)
      {
        motors::drive[i].setVelocity(0);
      }
    }
  }
  // printProfiles(profiles);
#endif // HAL_IMPLEMENTATION == HAL_REAL
}

// Helper Functions
#pragma region
// CALIBRATION
#if HAL_IMPLEMENTATION == HAL_REAL
void calMotor(SwerveCAN &can)
{
  for (int j = 0; j < kin.nWheels; j++)
  {
    steer[j]->setHoming(2); // set homing mode to true for all axes
    steer[j]->motTo(0, rcReceiver_getRedSwitch(), rcReceiver_rcLost());
  }
  delay(1000);           // Give motor time to move to zero position if it is wound up
  double fineTune = 1.0; // to step in less than 1 deg increments - this is the ratio (0.2 would be in 0.2 degree increments
  for (int i = 0; i < (int)(360 * std::abs(kin.yRatio) * 1.5 / fineTune); i++)
  {
    for (int j = 0; j < kin.nWheels; j++)
    {
      can.pos = i * fineTune;
      if (steer[j]->getHoming() == 2 && digitalRead(robotState.irPin[j]) == 1)
      { // calibration started with IR triggered
        steer[j]->setHoming(1);
      }
      if (steer[j]->getHoming() == 1 && digitalRead(robotState.irPin[j]) == 0)
      { // Hit target fresh
        planner->setSteerAngle(robotState.irPos[j] * PI / 180.0, j);
        planner->setMotAngle(can.pos * PI / 180.0, j);
        // steer[j]->setYaw(robotState.irPos[j]);
        // steer[j]->setMPos(can.pos);
        steer[j]->setHoming(0);
      }
      if (steer[j]->getHoming() == 0)
      { // position signal should persist, but motor should stop moving after cal marker is detected
        can.pos = steer[j]->getMPos();
      }
      steer[j]->motTo(can.pos * PI / 180.0, rcReceiver_getRedSwitch(), rcReceiver_rcLost());
    }
    delayMicroseconds(800);
    doneHoming = 1;
    for (int j = 0; j < kin.nWheels; j++)
    {
      doneHoming = doneHoming && steer[j]->getHoming() == 0;
    }
    if (doneHoming == 1)
    {
      kin.calibrated = 1; // set calibration flag to true
    }
    if (doneHoming == 1)
      // while (1){}  % used to freeze homing positions after cal to measure offsets with phone/level
      break; // break from loop once all targets have been found
  }
}
#else
void calMotor() {
  Serial.println("Motor calibration not supported in simulation");
}
#endif
double mapDouble(double x, double min_in, double max_in, double min_out, double max_out)
{
  double ret = (x - min_in) / (max_in - min_in);
  ret = ret * (max_out - min_out) + min_out;
  return ret;
}

void telemetry()
{
  unsigned long time = micros();
  if (time - prevTelemetryReportTime < TELEMETRY_REPORT_PERIOD)
  {
    return;
  }
  prevTelemetryReportTime = time;

  // Serial.println("\n");
}

#pragma endregion

#if HAL_IMPLEMENTATION == HAL_SIM
// Main function for simulation
#include <iostream>
#include <signal.h>
#include <chrono>
#include <thread>
#include <atomic>

// Global flag for clean shutdown
std::atomic<bool> g_running{true};

/**
 * @brief Signal handler for clean shutdown
 */
void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    g_running = false;
}

int main(int argc, char* argv[]) {
    // Set up signal handlers for clean shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "========================================" << std::endl;
    std::cout << "  Swerve Robot Simulation Mode" << std::endl;
    std::cout << "  HAL Implementation: " << HALFactory::getImplementationType() << std::endl;
    std::cout << "  Running swerve_master.ino" << std::endl;
    std::cout << "========================================" << std::endl;

    try {
        // Call robot setup (equivalent to Arduino setup())
        std::cout << "Running robot setup..." << std::endl;
        setup();

        std::cout << "Starting main loop..." << std::endl;
        std::cout << "Press Ctrl+C to stop" << std::endl;

        // Main loop (equivalent to Arduino loop())
        const auto targetLoopTime = std::chrono::microseconds(4500); // 4.5ms loop time

        while (g_running) {
            auto loopStart = std::chrono::steady_clock::now();

            // Run robot loop
            loop();

            // Maintain consistent loop timing
            auto loopEnd = std::chrono::steady_clock::now();
            auto loopDuration = loopEnd - loopStart;

            if (loopDuration < targetLoopTime) {
                std::this_thread::sleep_for(targetLoopTime - loopDuration);
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    // Clean shutdown
    std::cout << "Shutting down..." << std::endl;
    HAL::shutdown();
    std::cout << "Shutdown complete." << std::endl;

    return 0;
}
#endif // HAL_IMPLEMENTATION == HAL_SIM
