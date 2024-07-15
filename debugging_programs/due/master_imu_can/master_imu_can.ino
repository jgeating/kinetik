#include "DueCANLayer.h"

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

// IMU variables
double x = 0;
double y = 0;
double z = 0;
int bus = 1; // IMU bus
int IMU_ID = 22; //IMU node ID

void setup() {
  // Setup serial
  Serial.begin(115200);
  Serial.println("Starting up...");
  pinMode(13, OUTPUT);
  //analogReadResolution(anRes);

  delay(100);
  
  if(canInit(0, CAN_BPS_500K) == CAN_OK)
    Serial.print("CAN0: Initialized Successfully.\n\r");
  else
    Serial.print("CAN0: Initialization Failed.\n\r");  

  if(canInit(1, CAN_BPS_500K) == CAN_OK)
    Serial.print("CAN1: Initialized Successfully.\n\r");
  else
    Serial.print("CAN1: Initialization Failed.\n\r");  

  delay(1000);    // Give motor time to move
}

void loop() {

  // request IMU data
  //canTx(bus select, CAN ID, ext bool, data, length);
  stmp[0] = 0xA7; //A7 is Request for IMU
  //Serial.println("requesting IMU data...");
  canTx(  bus,          IMU_ID,   false,        stmp, 8);  
  //delay(100);
  delayMicroseconds(100);
  int buffed = 1;
  //Serial.println("Message sent...");
  while (buffed){
    if(canRx(bus, &lMsgID, &bExtendedFormat, &cRxData[0], &cDataLen) == CAN_OK)
    {
      if (lMsgID == 22){
        x = (int16_t)(cRxData[2] + (cRxData[3] << 8))/100.0;   
        y = (int16_t)(cRxData[4] + (cRxData[5] << 8))/100.0;    
        z = (int16_t)(cRxData[6] + (cRxData[7] << 8))/100.0;   
        /*
        Serial.print("CAN: Rx - MsgID:");
        Serial.print(lMsgID);
        Serial.print(" Ext:");
        Serial.print(bExtendedFormat);
        Serial.print(" Len:");
        Serial.print(cDataLen);
        Serial.print(" Data:");
        for(byte cIndex = 0; cIndex < cDataLen; cIndex++)
        {
          Serial.print(cRxData[cIndex], HEX);
          Serial.print(" ");
        }
        Serial.print("\n\r");
        */
        
        Serial.print("X: ");
        Serial.print(x);
        Serial.print(" Y: ");
        Serial.print(y);
        Serial.print(" Z: ");
        Serial.println(z);
        
      }
    } else {
      buffed = 0;
    }
  }
  // wait for response
  //delay(100);
}

void rxMsg(){
  if(canRx(1, &lMsgID, &bExtendedFormat, &cRxData[0], &cDataLen) == CAN_OK)
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
