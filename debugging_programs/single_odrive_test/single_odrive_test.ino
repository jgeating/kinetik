#include "DueCANLayer.h"
#include <math.h>
#include "utils.h"

//example python code from odrive: https://github.com/odriverobotics/ODriveResources/blob/master/examples/can_simple.py
//CAN packet structure for odrive: https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#can-protocol

extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte* cData, byte cDataLen);
extern byte canRx(byte cPort, long* lMsgID, bool* bExtendedFormat, byte* cData, byte* cDataLen);

extern byte canInit(byte cPort, long lBaudRate);

bool ext = false;
int mot = 2;

void setup() {
  Serial.begin(115200);
  delay(2000);
  if (canInit(1, CAN_BPS_1000K) == CAN_OK)
    Serial.print("CAN1: Initialized Successfully.\n\r");
  else
    Serial.print("CAN1: Initialization Failed.\n\r");
  
  if (canInit(0, CAN_BPS_1000K) == CAN_OK)
    Serial.print("CAN0: Initialized Successfully.\n\r");
  else
    Serial.print("CAN0: Initialization Failed.\n\r");

  int idd = mot << 5 | 0x07;  // idd is the id of the CAN packet
  ext = false;
  byte odrive_data[4] = { 0x08, 0x00, 0x00, 0x00 };

  canTx(0, idd, ext, odrive_data, sizeof(odrive_data));
  delay(1000);
}

void printMessage() {
  long lMsgID;
  bool bExtendedFormat;
  byte cRxData[8];
  byte cDataLen;
  if(canRx(0, &lMsgID, &bExtendedFormat, &cRxData[0], &cDataLen) == CAN_OK)
  {
    Serial.print("CAN0: Rx - MsgID:");
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
  }
}

void loop() {
  // delay(100);
  printMessage();
  float velocity = .7;
  float torqueFF = 0.0;

  byte odrive_data_hardcode[] = { 0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0x00 };
  byte odrive_data[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  byte odrive_data_big_endian[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  memcpy(odrive_data_big_endian, &velocity, sizeof(velocity));
  memcpy(odrive_data_big_endian + sizeof(velocity), &torqueFF, sizeof(torqueFF));

  // reverse order of bytes for little-endian
  // for (int m = 0; m < 4; m++) {
  //   // velocity
  //   odrive_data[m] = odrive_data_big_endian[3 - m];
  //   // torqueFF
  //   odrive_data[m + 4] = odrive_data_big_endian[7 - m];
  // }

  int idd = mot << 5 | 0x0d;  // 6/22/2021, added +1 because I indexed at 1, not 0, 4/9/2023, switched back, because of zero index

  canTx(0, idd, ext, odrive_data_big_endian, sizeof(odrive_data));
  delay(10);
}
