/**
 * @file robot_main.cpp
 * @brief Main robot functions extracted from Arduino code
 *
 * This file contains the main robot setup and loop functions that work
 * with both real hardware and simulation through the HAL.
 *
 * NOTE: This file is only compiled for simulation (HAL_SIM)
 */

#include "hal/HALConfig.h"

#if HAL_IMPLEMENTATION == HAL_SIM

#include "hal/HALFactory.h"
#include "hal/ArduinoCompat.h"

// Include existing robot headers
#include "Kinematics.h"
#include "Planner.h"
#include "shared/utils.h"
#include "penny/Steer.h"
#include "penny/Pads.h"
#include "penny/Drive.h"
#include "Swerve.h"
#include "Performance.h"
#include "PID.h"
#include "penny/Lights.h"
#include "SwerveTelemetry.h"

#if HAL_IMPLEMENTATION == HAL_REAL
#include "SbusReceiver.h"
#endif

// Robot constants
#define RADIUS_SWERVE_ASSEMBLY 0.25 // distance to wheel swerve axes, meters
#define DEAD_ZONE 0.1
#define pi 3.14159265358979
#define TELEMETRY_REPORT_PERIOD 500000
#define MCU "NATIVE" // Updated for HAL version

// Global robot state
SwerveTrajectory traj;
pad_vars padVars;
SwerveKinematics kin;
LoopTiming loopTiming;
Profiles profiles;
Modes modes; // Handles different driving and control modes

// Robot components
Planner *planner = nullptr; // for planning robot level motion
Pads *pads = nullptr;       // For driving with force pads
PID *padx_pid = nullptr;    // PID controller for weight (pad) steering in X (sideways) axis
PID *pady_pid = nullptr;    // PID controller for weight (pad) steering in Y (forwards) axis
PID *padz_pid = nullptr;    // PID controller for weight (pad) steering in Z (rotation) axis
Lights *lights = nullptr;   // Controls LED strips for signals/entertainment

// HAL-based components will be created through factory

// Robot state
RobotState robotState;
Watchdog watchdog;

#if HAL_IMPLEMENTATION == HAL_REAL
SbusReceiver sbusReceiver;
#endif

/**
 * @brief Robot setup function (equivalent to Arduino setup())
 */
void robotSetup() {
    // Initialize HAL components
    HAL::serial->begin(460800);
    HAL::serial->println("Starting robot setup...");

    // Initialize CAN bus
    if (HAL::canBus->begin(1000000)) {
        HAL::serial->println("CAN bus initialized successfully");
    } else {
        HAL::serial->println("CAN bus initialization failed");
    }

    // Initialize IMU
    if (HAL::imu->begin()) {
        HAL::serial->println("IMU initialized successfully");
        HAL::imu->setExternalCrystal(true);
    } else {
        HAL::serial->println("IMU initialization failed");
    }

    // Initialize RC receiver
    if (HAL::rcReceiver->begin()) {
        HAL::serial->println("RC receiver initialized successfully");
    } else {
        HAL::serial->println("RC receiver initialization failed");
    }

    // Initialize robot components
    planner = new Planner(2500.0, traj, padVars, kin); // tInner, traj, padVars, kin
    pads = new Pads(); // No parameters

    // Initialize PID controllers (Kp, Ki, Kd, sampleTime, lagFilterSize)
    padx_pid = new PID(0.5, 0.0, 0.1, 0.0045, 10); // 4.5ms sample time
    pady_pid = new PID(0.5, 0.0, 0.1, 0.0045, 10);
    padz_pid = new PID(0.5, 0.0, 0.1, 0.0045, 10);

    // Initialize lights (strip_count, led_pin)
    lights = new Lights(4, 2); // Example: 4 strips, pin 2

    // Set initial robot state (using existing struct members)
    modes.mode = Mode::TELEOP;
    modes.eStop = false;

    HAL::serial->println("Robot setup complete!");
}

/**
 * @brief Robot main loop function (equivalent to Arduino loop())
 */
void robotLoop() {
    static unsigned long lastLoopTime = 0;
    unsigned long currentTime = micros();

    // Update timing
    loopTiming.now = currentTime;
    lastLoopTime = currentTime;

    // Update HAL components
    HAL::rcReceiver->update();

    // Read RC input
    double leftVert = HAL::rcReceiver->getLeftVertical();
    double leftHor = HAL::rcReceiver->getLeftHorizontal();
    double rightVert = HAL::rcReceiver->getRightVertical();
    double rightHor = HAL::rcReceiver->getRightHorizontal();
    int blueSwitch = HAL::rcReceiver->getBlueSwitch();
    int redSwitch = HAL::rcReceiver->getRedSwitch();

    // Check for signal loss
    bool signalLost = HAL::rcReceiver->isSignalLost();
    modes.eStop = signalLost || (redSwitch == 0);

    // Read IMU data
    Vector3D eulerAngles = HAL::imu->getEulerAngles();
    Vector3D angularVel = HAL::imu->getAngularVelocity();

    // Update robot state with IMU data (using existing struct members)
    // Note: RobotState doesn't have yaw/yawRate, so we'll store in trajectory
    // or create local variables for now

    // Simple motion control based on RC input
    if (!modes.eStop) {
        // Convert RC input to robot motion
        double vx = leftVert * 2.0;  // Forward/backward velocity (m/s)
        double vy = leftHor * 2.0;   // Left/right velocity (m/s)
        double omega = rightHor * 2.0; // Rotation rate (rad/s)

        // Apply deadzone
        if (abs(vx) < DEAD_ZONE) vx = 0;
        if (abs(vy) < DEAD_ZONE) vy = 0;
        if (abs(omega) < DEAD_ZONE) omega = 0;

        // Update trajectory (using actual struct members)
        traj.qd_d[0] = vx;    // X velocity
        traj.qd_d[1] = vy;    // Y velocity
        traj.qd_d[2] = omega; // Angular velocity

        // Plan motion - Planner doesn't have update() method
        // We would call appropriate planning functions here
        // For now, just store the desired velocities

        // Send motor commands via CAN
        // This would normally send commands to the drive and steer motors
        // For now, we'll just simulate the CAN traffic

        // Update lights based on mode
        // Lights class doesn't have update() method, so we'll skip for now
    } else {
        // E-stop active - stop all motion
        traj.qd_d[0] = 0;
        traj.qd_d[1] = 0;
        traj.qd_d[2] = 0;

        // Emergency mode for lights would go here
    }

    // Telemetry output (every second)
    static unsigned long lastTelemetryTime = 0;
    if (currentTime - lastTelemetryTime > TELEMETRY_REPORT_PERIOD) {
        HAL::serial->print("Yaw: ");
        HAL::serial->print(eulerAngles.z, 2);
        HAL::serial->print(", Vx: ");
        HAL::serial->print(traj.qd_d[0], 2);
        HAL::serial->print(", Vy: ");
        HAL::serial->print(traj.qd_d[1], 2);
        HAL::serial->print(", Omega: ");
        HAL::serial->print(traj.qd_d[2], 2);
        HAL::serial->print(", E-stop: ");
        HAL::serial->println(modes.eStop ? "ACTIVE" : "CLEAR");

        lastTelemetryTime = currentTime;
    }

    // Update watchdog
    watchdog.prevLoopTime = currentTime;
}

/**
 * @brief Cleanup function called on shutdown
 */
void robotShutdown() {
    HAL::serial->println("Shutting down robot...");

    // Clean up dynamically allocated objects
    delete planner;
    delete pads;
    delete padx_pid;
    delete pady_pid;
    delete padz_pid;
    delete lights;

    planner = nullptr;
    pads = nullptr;
    padx_pid = nullptr;
    pady_pid = nullptr;
    padz_pid = nullptr;
    lights = nullptr;

    HAL::serial->println("Robot shutdown complete");
}

#endif // HAL_IMPLEMENTATION == HAL_SIM
