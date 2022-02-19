#include "DueCANLayer.h"

extern byte canInit(byte cPort, long lBaudRate);
extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte* cData, byte cDataLen);
extern byte canRx(byte cPort, long* lMsgID, bool* bExtendedFormat, byte* cData, byte* cDataLen);

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


void setup() {
  // Setup serial
  Serial.begin(115200);
  Serial.println("Starting up...");
  pinMode(13, OUTPUT);
  //analogReadResolution(anRes);

  if(canInit(0, CAN_BPS_1000K) == CAN_OK)
    Serial.print("CAN0: Initialized Successfully.\n\r");
  else
    Serial.print("CAN0: Initialization Failed.\n\r");  

  if(canInit(1, CAN_BPS_1000K) == CAN_OK)
    Serial.print("CAN1: Initialized Successfully.\n\r");
  else
    Serial.print("CAN1: Initialization Failed.\n\r");  


  Serial.println("Sending motor to starting position");
  ang = 0;
  stmp[4] = *((uint8_t *)(&ang));
  stmp[5] = *((uint8_t *)(&ang)+1);
  stmp[6] = *((uint8_t *)(&ang)+2);
  stmp[7] = *((uint8_t *)(&ang)+3);
  stmp[0] = 0xA4;
  stmp[3] = 0x03;
  canTx(0, 0, false, stmp, 8);
  stmp[0] = 0xA3;
  stmp[3] = 0x00;
  delay(1000);    // Give motor time to move
}

void loop() {
  ang = 0;
  stmp[4] = *((uint8_t *)(&ang));
  stmp[5] = *((uint8_t *)(&ang)+1);
  stmp[6] = *((uint8_t *)(&ang)+2);
  stmp[7] = *((uint8_t *)(&ang)+3);
  
  //Serial.println("Sending data");
  canTx(  0,          0,   false,        stmp, 8);    
  //canTx(bus select, CAN ID, ext bool, data, length);
  delay(100);
  Serial.println("******************");
  Serial.print("CAN receive status (1 = success, 0 = fail): ");
  Serial.println(canRx(1, &rxlMsgID, &rxbExtendedFormat, &cRxData[0], &rxcDataLen));
  Serial.print("ID: ");
  Serial.println(rxlMsgID);
  Serial.print("Data: ");
  for (int i = 0; i < rxcDataLen; i++){
    Serial.print(cRxData[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  delay(500);
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
