#ifndef _PADS_
#define _PADS_

// Force Sensor Variables
#define forceMode 0  // Defines if the vehicle will be steered in force control mode 0 = false
#define numForceSensors 8


class Pads
{
  private:
    int forcepins[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int fvolts[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int forcezeros[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    double forces[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    double xgain;
    double ygain;
    double zgain;
    double vMax;
    double wMax;
    double totalweight = 0;
    double weightthresh;
    double fzeros[8] = {0, 0, 0, 0, 0, 0, 0, 0};  //Zeros for calibratin to rider center of pressure
    double yvel = 0;
    double xvel = 0;
    double spinvel = 0;
    double capacity = 50*9.81;  // capacity of load cell, in N
    double mvv = 1;             // mv/V value of the load cell
    double ampgain = 495;       // amplifier gain, unitless
    double intToN;

    void getForces();
    void getRawForces();
  public:
    Pads(int forcepins[], int forcezeros[], double xgain, double ygain, double zgain, double vMax, double wMax, double weightthresh);
    void calcVector();
    void calibrate();
    bool fallDetected();   
    double getForce(int ch);
    double getYVel();
    double getXVel();
    double getSpinVel();
}; 


#endif
