#ifndef _PADS_
#define _PADS_

#include "Arduino.h"

// Force Sensor Variables
#define forceMode 1  // Defines if the vehicle will be steered in force control mode 0 = false
#define numForceSensors 8



  //                  0    1    2    3    4    5    6    7
  //Corresponds to  {RY+, RY-, RX+, RX-, LY+, LY-, LX+, LX-}
  //Values are:     {RFL, RFR, RBL, RBR, LFL, LFR, LBL, LBR} (i.e. Left back right = left pad, in the back right corner)

class Pads
{
  private:
    int forcepins[8] =  { A15, A14, A17, A16, A1, A0, A3, A2 };   // Sets force pads. See description above 
    int fvolts[8] =     {0, 0, 0, 0, 0, 0, 0, 0};
    int analogZeros[8] = { 115, 153, 176, 154, 193, 36, 173, 286 }; // zero point in ADC counts of sensor. Does not get calibrated during runtime
    double forces[8] =  {0, 0, 0, 0, 0, 0, 0, 0};
    double totalweight = 0;
    double weightthresh = 100;
    double fzeros[8] = {0, 0, 0, 0, 0, 0, 0, 0};  //Zeros for calibratin to rider center of pressure - not ADC counts, actual Newtons 
    double y_out = 0;
    double x_out = 0;
    double z_out = 0;
    double capacity = 50*9.81;  // capacity of load cell, in N
    double mvv = 1;             // mv/V value of the load cell
    double ampgain = 495;       // amplifier gain, V/V
    double intToN;
    double cart[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // cartesian forces, combining corner forces into F/R/L/R directions 

    void getForces();
    void getRawForces();
  public:
    Pads();
    void calcVector();
    void calibrate();
    bool fallDetected();   
    double getForce(int ch);
    double getY();
    double getX();
    double getZ();
    void printDebug();
}; 

#endif
