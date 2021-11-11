#ifndef _PLANNER_
#define _PLANNER_

// Robot level planning
class Planner
{
  private:
    double dt;        // length of time step, microseconds

    double qd_x_d;    // desired x velocity, m/s
    double qd_y_d;    // desired y velocity, m/s
    double qd_z_d;    // desired w velocity, rad/sec
    double q_z_d;     // desired orientation, rad

    double qd_x;      // current x velocity, m/s
    double qd_y;      // current y velocity, m/s
    double qd_z;      // current yaw velocity, rad/sec
    double q_z;       // current orientation, rad

    double qd_x_max;  // max x velocity allowed, m/sec
    double qd_y_max;  // max y velocity allowed, m/sec
    double qd_z_max;  // max yaw velocity allowed, rad/sec
    double qdd_x_max; // max x acceleration allowed, m/s^2   
    double qdd_y_max; // max y acceleration allowed, m/s^2
    double qdd_z_max; // max yaw acceleration allowed, rad/s^2

    double x_dead;  // dead zone in x axis, m/s. Robot will not move if desired is slower than this
    double y_dead;  // dead zone in y axis, m/s. Robot will not move if desired is slower than this
    double z_dead;  // dead zone in w axis, m/s. Robot will not move if desired is slower than this

    double del_qd_x_max;
    double del_qd_y_max;
    double del_qd_z_max;

    int sign(double x);
    double lim(double x, double a, double b);
    
  public:
    Planner(double dt, double qd_x_max, double qd_y_max, double qd_z_max, double qdd_x_max, double qdd_y_max, double qdd_z_max, double x_dead, double y_dead, double z_dead);
    void calc(double x_in, double y_in, double z_in);
    double getTargetVX();
    double getTargetVY();
    double getTargetVZ();
};

#endif
