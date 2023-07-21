#include "DueCANLayer.h"
#include <math.h>
#include "utils.h"

//example python code from odrive: https://github.com/odriverobotics/ODriveResources/blob/master/examples/can_simple.py
//CAN packet structure for odrive: https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#can-protocol

extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte* cData, byte cDataLen);
extern byte canInit(byte cPort, long lBaudRate);

byte cTxData1[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte odrive_data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08};
int idd = 1 || CAN_PACKET_SET_RPM << 8; // Drive ID
bool ext = true; 
int len = 8;
uint32_t pos = 0;
// For receiving 
short rxData = 0;
long lMsgID;
bool bExtendedFormat;
byte cRxData[8];
byte cDataLen;
int mot = 0;

void setup() {
  delay(2000);
  if(canInit(1, CAN_BPS_1000K) == CAN_OK)
    Serial.print("CAN1: Initialized Successfully.\n\r");
  else
    Serial.print("CAN1: Initialization Failed.\n\r"); 

  mot = 2;
  int idd = mot << 5 | 0x07; // idd is the id of the CAN packet
  ext = false; 
  // odrive_data[7] = 0x08;
  canTx(1, idd, ext, odrive_data, 8);
  delay(1000);
}

void loop() {
  delay(100);
  float vels[] = {0.7, 0.7, 0.7, 0.7};
  bool ext = true;

  if (0){
    for (int k = 0; k < 4; k++) {
      int vel = vels[k];
      mot = k;
      for (int m = 0; m < len; m++) {
        cTxData1[len - m -1] = (int)vel >> 8 * m;
      }
      int idd = mot | CAN_PACKET_SET_RPM << 8;
      canTx(1, idd, ext, cTxData1, len);
      delay(10);
    }
  } else {
    int k = 2;
    int vel = vels[k];
    mot = k;
    len = 8;
    // Send TFF data
    // for (int m = 0; m < 4; m++) {
    //   odrive_data[len - m -1] = (int)vel >> 8 * m;
    // }
    // // Send Vel data
    // for (int m = 4; m < 8; m++) {
    //   odrive_data[len - m - 1] = (int)vel >> 8 * m;
    // }

    float velocity = 0.7;
    float torqueFF = 0.0;

    memcpy(odrive_data, &velocity, sizeof(velocity));
    memcpy(odrive_data + sizeof(velocity), &torqueFF, sizeof(torqueFF));

    ext = false;

    int idd = mot << 5 | 0x0d;  // 6/22/2021, added +1 because I indexed at 1, not 0, 4/9/2023, switched back, because of zero index
    
    canTx(1, idd, ext, odrive_data, len);
    delay(10);
  }
}
