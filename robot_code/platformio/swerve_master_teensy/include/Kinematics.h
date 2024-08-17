#ifndef _KINEMATICS_
#define _KINEMATICS_

class Kinematics
{
  private:
    int DEAD_ZONE;
    double xSteerCoeff;
    double ySteerCoeff;
    double targetSteer;
    double targetVel;
    double r;
  public:
    Kinematics(double RADIUS_SWERVE_ASSEMBLY, int DEAD_ZONE, int wheelIndex);
    void calc(double vx, double vy, double vz);
    double getTargetSteer();
    double getTargetVel();
}; 

#endif
