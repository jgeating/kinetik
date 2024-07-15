#include "DueCANLayer.h"

// CAN Layer functions
extern byte canInit(byte cPort, long lBaudRate);
extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte* cData, byte cDataLen);
extern byte canRx(byte cPort, long* lMsgID, bool* bExtendedFormat, byte* cData, byte* cDataLen);

void setup()
{
  // Set the serial interface baud rate
  Serial.begin(115200);
  
  // Initialize both CAN controllers
  if(canInit(0, CAN_BPS_1000K) == CAN_OK)
    Serial.print("CAN0: Initialized Successfully.\n\r");
  else
    Serial.print("CAN0: Initialization Failed.\n\r");
  
  if(canInit(1, CAN_BPS_1000K) == CAN_OK)
    Serial.print("CAN1: Initialized Successfully.\n\r");
  else
    Serial.print("CAN1: Initialization Failed.\n\r");

  // Declarations
  byte cTxData0[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  byte cTxData1[] = {0x0F, 0x0E, 0x0D, 0x0C, 0x0C, 0x0B, 0x0A, 0x08};
  int nTimer0 = 0;
  int nTimer1 = 0;
}// end setup

void loop()
{
  while(1){
    //Serial.println("...");
    delay(1);
      // Check for received message
    long lMsgID;
    bool bExtendedFormat;
    byte cRxData[8];
    byte cDataLen;
    if(canRx(0, &lMsgID, &bExtendedFormat, &cRxData[0], &cDataLen) == CAN_OK)
    {
      Serial.print("CAN1: Rx - MsgID:");
      Serial.print(lMsgID);
      Serial.print(" Node ID:");
      Serial.print(lMsgID >> 5);
      Serial.print(" Control ID:");
      Serial.print(lMsgID & 31);
      Serial.print(" Ext:");
      Serial.print(bExtendedFormat);
      Serial.print(" Len:");
      Serial.print(cDataLen);
      Serial.print(" Data:");

      for(byte cIndex = 0; cIndex < cDataLen; cIndex++)
      {
        Serial.print(cRxData[cIndex], HEX);
        Serial.print(" ");
      }// end for

      Serial.print("\n\r");
      Serial.println(lMsgID == (2 << 5 | 0x01));
      
    }// end if
  }
}// end loop
