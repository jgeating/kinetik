#ifndef _PLANNER_
#define _PLANNER_

// Robot level planning
class Planner
{
  private: 
    // safety, timing, state related
    int mode;         // mode for what inputs/control algorithm to use
    double dt;        // length of time step, microseconds
    double dband_teleop[3];  // Deadband of controller when in teleop mode. In fraction of total command (0 to 1)

    // Vest related
    double in0[3];    // zeros for input angles from IMU(s), rad
    double input[3];  // input for various control modes, units dependent on control mode
  
    // Kinematics/trajectory related 
    double q[3];      // Current position/orientation, x/y/yaw, m or rad
    double qd[3];     // Current velocity, m/s or rad/s
    double qdd[3];    // current acceleration, m/s^2 or rad/s^2
    double q_d[3];    // desired position, m or rad
    double qd_d[3];   // desired velocity, m/s or rad/s
    double qdd_d[3];  // Desired acceleration, m/s^2 or rad/s^2
    double qd_max[3]; // max velocity allowed, m/s or rad/s
    double qdd_max[3];// max acceleration allowed, m/s^2 or rad/s^2
    double dband[3];  // dead band zone, in m/s or rad/s. robot will not move if desired is slower than this 
    double del_qd_max[3];   // intermediate variable for calculations
    double alpha = 0; // angle of robot wrt world frame, rad

    // Functions
    int calcFromAccels();                     // for planning in acceleration control
    int calcFromVels();                       // main private planning function
    double sign(double x);                       // returns the sign of the number, either 1 or -1
    double lim(double x, double a, double b); // arduino contrain port, for doubles 
    double dewrap(double x);
    
  public:
    Planner(double tInner, 
            double qd_x_max, double qd_y_max, double qd_z_max, 
            double qdd_x_max, double qdd_y_max, double qdd_z_max, 
            double x_dead, double y_dead, double z_dead, 
            double x_dead_t, double y_dead_t, double z_dead_t, 
            int mode
            );
    
    // Main functions 
    int plan(double x_in, double y_in, double z_in);
    int plan_world(double x_in, double y_in, double z_in, double alpha);

    // Setters, getters
    void setMode(int mode);
    void setZeros(double x_in, double y_in, double z_in);
    double getTargetVX();
    double getTargetVY();
    double getTargetVZ();
};

#endif
