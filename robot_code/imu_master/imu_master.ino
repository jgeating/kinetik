#include "DueCANLayer.h"
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"

#define BNO055_SAMPLERATE_DELAY_MS (10)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

extern byte canInit(byte cPort, long lBaudRate);
extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte* cData, byte cDataLen);
extern byte canRx(byte cPort, long* lMsgID, bool* bExtendedFormat, byte* cData, byte* cDataLen);

// CAN Stuff
byte cTxData0[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte cTxData1[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char len = 0;
unsigned char buf[8];
unsigned char stmp[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
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
long canID = 22; //IMU node ID

void setup() {
  // Setup serial
  Serial.begin(115200);
  Serial.println("Starting up...");
  pinMode(13, OUTPUT);
  //analogReadResolution(anRes);

  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(100);

    bno.setExtCrystalUse(true);

  if(canInit(0, CAN_BPS_500K) == CAN_OK)
    Serial.print("CAN1: Initialized Successfully.\n\r");
  else
    Serial.print("CAN1: Initialization Failed.\n\r");  
  if(canInit(1, CAN_BPS_500K) == CAN_OK)
    Serial.print("CAN1: Initialized Successfully.\n\r");
  else
    Serial.print("CAN1: Initialization Failed.\n\r");  


  delay(1000);    // Give motor time to move
}

void loop() {
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  // check for IMU request from master
  // Check for received message
  //if(canRx(0, &lMsgID, &bExtendedFormat, &cRxData[0], &cDataLen) == CAN_OK)
  //{
    // if request was master requesting IMU data (0xA7)
    //if(lMsgID == canID && cRxData[0] == 0xA7)
    //{
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      
      /* Display the floating point data */
      
      Serial.print("X: ");
      Serial.print(euler.x());
      Serial.print(" Y: ");
      Serial.print(euler.y());
      Serial.print(" Z: ");
      Serial.println(euler.z());
      delay(20);
      
      //delayMicroseconds(1000);
      
      ang = 100.0 * euler.x();
      stmp[2] = *((uint8_t *)(&ang));
      stmp[3] = *((uint8_t *)(&ang)+1);
      ang = 100.0 * euler.y();
      stmp[4] = *((uint8_t *)(&ang));
      stmp[5] = *((uint8_t *)(&ang)+1);
      ang = 100.0 * euler.z();
      stmp[6] = *((uint8_t *)(&ang));
      stmp[7] = *((uint8_t *)(&ang)+1);
      canTx(0, canID, false, stmp, 8);
      delayMicroseconds(50);
      canTx(1, canID, false, stmp, 8);
    //}
  //}
}
