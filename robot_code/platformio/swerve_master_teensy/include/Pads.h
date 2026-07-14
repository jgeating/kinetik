#ifndef _PADS_
#define _PADS_

// Force Sensor Variables
#define forceMode 1  // Defines if the vehicle will be steered in force control mode 0 = false
#define numForceSensors 8

class Pads
{
  //                         0,    1,    2,    3,    4,    5,    6,    7
  //Corresponds to        {RY+,  RY-,  RX+,  RX-,  LY+,  LY-,  LX+,  LX-}
  //Values are:           {RFL,  RFR,  RBL,  RBR,  LFL,  LFR,  LBL,  LBR} (i.e. Left back right = left pad, in the back right corner)
  private:
    // Hardware configuration
    double weightthresh = 100;  // Threshold under which we consider the rider to have fallen off, in Newtons
    double capacity = 50*9.81;  // capacity of load cell, in N
    double mvv = 1;             // mv/V value of the load cell
    double ampgain = 495;       // amplifier gain, V/V 
    double cal_weight = 24.1/2.2*9.81; // Calibration weight used to determine scaling factor of load cellsl, in N
    double x_out_scale = 250; // COP location (left/right) corresponding to full scale, mm
    double y_out_scale = 250; // COP location (fore/aft) corresponding to full scale, mm
    double z_out_scale = 250; // COP location (rotational) corresponding to full scale, mm
    
    int forcepins[8] =   {  17,   16,   15,   14,   41,   40,   39,   38}; // Sets force pad pins. See description above 
    double x_loc[8] =    { 305,   20,   20,  305,  -20, -305,  -20, -305}; // x location (right = +) of each sensor, relative to center of the robot
    double y_loc[8] =    {-130, -307,  130,  130,  130,  130, -307, -130}; // y location (front = +) of each sensor, relative to center of the robot
    int analogZeros[8] = { 118,   87,   91,  206,  150,  251,   94,   45}; // zero point in ADC counts of sensor. Does not get calibrated during runtime. Spring preload calculated by subtracting from these 
    int calADCcnts[8] =  {1550, 1448,  960, 1485, 1500, 1630, 1480,  914}; // ADC counts corresponding to calibration weight. Used to determine scaling factor of load cells. Zero offset not accounted for (not tared)
    double scaling[8] =  {   0,    0,    0,    0,    0,    0,    0,    0}; // scaling factor from ADC counts to Newtons for each sensor, determined by calibration weight and corresponding ADC counts

    // State variables
    int fvolts[8] =      {   0,    0,    0,    0,    0,    0,    0,    0}; //raw voltage reading from the pads, ADC counts
    double rawforces[8] ={   0,    0,    0,    0,    0,    0,    0,    0}; // raw forces in Newtons calculated from ADC counts and scaling factor, not accounting for zero offset
    double forces[8] =   {   0,    0,    0,    0,    0,    0,    0,    0}; // Pad forces, N etc. 
    double fzeros[8] =   {   0,    0,    0,    0,    0,    0,    0,    0}; // Zeros for calibrating preload force, ADC counts 
    double ftare[8] =    {   0,    0,    0,    0,    0,    0,    0,    0}; // Tare values for zeroing out forces when rider is on board, in Newtons
    double cart[8] =     {   0,    0,    0,    0,    0,    0,    0,    0}; // cartesian forces, combining corner forces into F/R/L/R directions 
    // double intToN;
    double totalweight = 0;
    double cop_x_l = 0; // center of pressure x coordinate for left pad, in mm
    double cop_x_r = 0; // center of pressure x coordinate for right pad, in mm
    double cop_y_l = 0; // center of pressure y coordinate for left pad, in mm
    double cop_y_r = 0; // center of pressure y coordinate for right pad, in mm
    double cop_x = 0; // center of pressure x coordinate, in mm
    double cop_y = 0; // center of pressure y coordinate, in mm
    double cop_z = 0; // center of pressure z coordinate (rotational component), in virtual mm
    double x_out = 0; // output signal for left/right control, scaled from -1 to 1
    double y_out = 0; // output signal for fore/aft control, scaled from -1 to 1
    double z_out = 0; // output signal for rotational control, scaled from -1 to 1

    void getForces();
    void getRawForces();
  public:
    Pads();
    void calcVector();
    void calibrate();
    void tare();
    bool fallDetected();   
    double getForce(int ch);
    double getRawForce(int ch);
    double getTotalWeight();
    double getY();
    double getX();
    double getZ();

    double get_cop_x();
    double get_cop_y();
    double get_cop_z();
    double get_cop_x_l();
    double get_cop_y_l();
    double get_cop_x_r();
    double get_cop_y_r();
    void printDebug();
}; 

#endif
