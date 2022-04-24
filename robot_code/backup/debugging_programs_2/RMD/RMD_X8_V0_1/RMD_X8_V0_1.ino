#include <math.h>
#include <Wire.h> 
#include "DueCANLayer.h"
#include "LiquidCrystal_I2C.h"

extern byte canInit(byte cPort, long lBaudRate);
extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte* cData, byte cDataLen);
extern byte canRx(byte cPort, long* lMsgID, bool* bExtendedFormat, byte* cData, byte* cDataLen);

// LCD stuff
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
char line1[16];
char line2[16];
String line1str = "";
String line2str = "";

// User input variables: for life test stand
double freq = 1;        // target oscillation frequency, Hz
double w_max = 720;     // max angular speed, deg/sec
double theta = 120;     // half amplitude of oscillation, degrees
double tInner = 2.5;    // Target length of inner loop controller, milliseconds
double tOuter = 100.0;  // Target length of outer loop controller, milliseconds
double a_max = 1800;    // Max angular acceleration for slew trajectory, deg/sec^2
double osc_ang = 180;   // oscillation angle, degrees
double osc_var = .75;    // variation in oscillation angle, unitless. 0.1 = 10% variation 
int ncycles = 5000;       // number of cycles to stop after

// CAN Stuff
byte cTxData0[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte cTxData1[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char len = 0;
unsigned char buf[8];
unsigned char stmp[8] = {0xA3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int32_t ang = 0;
bool ext = true; 
uint32_t pos = 0;
// For receiving 
short rxData = 0;
//Tx
long lMsgID;
bool bExtendedFormat;
byte cRxData[8];
byte cDataLen;
//Rx
long rxlMsgID;
bool rxbExtendedFormat;
byte rxcDataLen;

// Loop Timing Variables
unsigned long lastInner = 0;          // last time counter of inner loop
unsigned long lastOuter = 0;          // last time counter of outer loop
unsigned long now = 0;                // variable to store current time, to avoid multiple calls to micros() function
bool behind = 0;                      // If Due can't keep up with loop rate, this bool will become true
double cnt = 0;                       // counter for looping through frequency
double t_w_max; 
double fInner;
double dp_max = 0;
double wp_max = 0;  
int loop_len = 0; 

// trajectory variables
int end_code = 0; // code used to flag when we're at the beginning or end of a slew. For output etc. 
int dir = 1;      // which direction we're currently going in
double plus_target = osc_ang/2;   // positive randomized trajectory target
double minus_target = -osc_ang/2;
double ratio = 6;
double input = 0; // -1 to 1 tracking variable
int cycle = 0;  // which cycle we're on
double current_plus = 0; // current in the plus rotation direction, amps
double current_minus = 0; 

void setup()
{
  // Compute timing parameters
  tInner = tInner / 1000;   // convert to seconds
  tOuter = tOuter / 1000;   // convert to seconds
  fInner = 1/tInner;        // Computer inner loop frequency
  dp_max = w_max / fInner;  // Max delta_pos per dt to stay within max angular velocity
  wp_max = a_max / fInner;  // max delta_vel per dt to stay within max angular acceleration
  loop_len = floor(fInner / freq);

  // Setup LCD
  lcd.init();
  lcd.backlight();

  // Setup serial
  Serial.begin(115200);
  Serial.println("Starting up...");
  pinMode(13, OUTPUT);
  //analogReadResolution(anRes);
  
  if(canInit(0, CAN_BPS_1000K) == CAN_OK)
    Serial.print("CAN0: Initialized Successfully.\n\r");
  else
    Serial.print("CAN0: Initialization Failed.\n\r");  

  Serial.println("Sending motor to starting position");
  ang = -osc_ang * 100 * ratio / 2;
  stmp[4] = *((uint8_t *)(&ang));
  stmp[5] = *((uint8_t *)(&ang)+1);
  stmp[6] = *((uint8_t *)(&ang)+2);
  stmp[7] = *((uint8_t *)(&ang)+3);
  stmp[0] = 0xA4;
  stmp[3] = 0x03;
  canTx(0, 0x141, false, stmp, 8);
  stmp[0] = 0xA3;
  stmp[3] = 0x00;
  delay(3000);    // Give motor time to move
}

void loop()
{
  now = micros();
  if (cycle < ncycles) {
    if (now - lastInner > tInner*1000000){
      if (behind){ // This means we can't keep up with the desired loop rate. Trip LED to indicate so (indicating over serial is too slow)
        digitalWrite(13, HIGH);
        lastInner = now;
      } else {
        digitalWrite(13, LOW);
        lastInner = lastInner + tInner*1000000;
        behind = true;
      }
  
      // *********************** BEGIN INNER LOOP ***************************
      
      cnt = cnt + 1;
      if (cnt == loop_len){
        plus_target = osc_ang/2 + osc_ang/2*random(-osc_var*1000, osc_var*1000)/1000;
        cnt = 0;
        cycle = cycle + 1;
        sprintf(line1, "%d/%d ", cycle, ncycles);
        lcd.setCursor(0,0); // set the cursor to column 15, line 0
        lcd.print(line1);

        sprintf(line2, "%.3fA/%.3fA  ", current_plus, current_minus);
        lcd.setCursor(0,1); // set the cursor
        lcd.print(line2);
      
      }
      
      if (cnt == floor(loop_len/2)){
        minus_target = -osc_ang/2 + osc_ang/2*random(-osc_var*1000, osc_var*1000)/1000;
      }
      
      input = cnt / loop_len;
      input = input * 3.141592 * 2;
      input = cos(input);
      ang = map_double(input, -1, 1, plus_target*100*ratio, minus_target*100*ratio);
  
      stmp[4] = *((uint8_t *)(&ang));
      stmp[5] = *((uint8_t *)(&ang)+1);
      stmp[6] = *((uint8_t *)(&ang)+2);
      stmp[7] = *((uint8_t *)(&ang)+3);
      
      //Serial.println("Sending data");
      canTx(  0,          0x141,   false,        stmp, 8);    
      //canTx(bus select, CAN ID, ext bool, data, length);

      if (cnt == floor(loop_len/4)){
        canRx(0, &rxlMsgID, &rxbExtendedFormat, &cRxData[0], &rxcDataLen);
        current_plus = map_double((uint8_t)(cRxData[2] + (cRxData[3] << 8)), 0, 2048, 0, 33);
      }

      if (cnt == floor(loop_len*3/4)){
        canRx(0, &rxlMsgID, &rxbExtendedFormat, &cRxData[0], &rxcDataLen);
        current_minus = map_double((uint8_t)(cRxData[2] + (cRxData[3] << 8)), 0, 2048, 0, 33);
      }
  
      
  
    //END INNER LOOP *******************************************************
    } else {
      behind = false;
    }
  } else { // when done, move motor back to zero
    Serial.println("Sending motor to zero");
    ang = 0;
    stmp[4] = *((uint8_t *)(&ang));
    stmp[5] = *((uint8_t *)(&ang)+1);
    stmp[6] = *((uint8_t *)(&ang)+2);
    stmp[7] = *((uint8_t *)(&ang)+3);
    stmp[0] = 0xA4;
    stmp[3] = 0x03;
    canTx(0, 0x141, false, stmp, 8);
    stmp[0] = 0xA3;
    stmp[3] = 0x00;
    delay(3000);    // Give motor time to move
  }

  if (now - lastOuter > tOuter){
    lastOuter = lastOuter + tOuter;

    //***********************BEGIN OUTER LOOP*******************************    

  }
}

// *********************************************** HELPER FUNCTIONS **************************************************************

// ************************* CAN RECEIVER
void rxMsg(){
  if(canRx(0, &lMsgID, &bExtendedFormat, &cRxData[0], &cDataLen) == CAN_OK)
  {
    rxData = 0;
    for (int i = 0; i < len; i++){
      rxData = rxData << 8;
      rxData |=  cRxData[i];
    }
  }
}

double map_double(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
