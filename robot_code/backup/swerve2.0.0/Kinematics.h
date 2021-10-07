#ifndef _KINEMATICS_
#define _KINEMATICS_

class Kinematics
{
  private:
    int DEAD_ZONE;
    double xYawCoeff;
    double yYawCoeff;
    double velGain; // 30 for humans driving
    double yawGain; // 60 for safe driving
    double targetYaw;
    double targetRPM;
  public:
    Kinematics(double velGain, double yawGain, double RADIUS_SWERVE_ASSEMBLY, int DEAD_ZONE, int wheelIndex);
    void calc(int xChannel, int yChannel, int zChannel);
    double getTargetYaw();
    double getTargetRPM();
}; 


#endif
