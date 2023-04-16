#include "DueCANLayer.h"
#include <math.h>
#include "utils.h"

extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte* cData, byte cDataLen);
extern byte canInit(byte cPort, long lBaudRate);

byte cTxData1[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int idd = 1 || CAN_PACKET_SET_RPM << 8; // Drive ID
bool ext = true; 
int len = 4;
uint32_t pos = 0;
// For receiving 
short rxData = 0;
long lMsgID;
bool bExtendedFormat;
byte cRxData[8];
byte cDataLen;
int mot = 0;

void setup() {
if(canInit(1, CAN_BPS_1000K) == CAN_OK)
    Serial.print("CAN1: Initialized Successfully.\n\r");
  else
    Serial.print("CAN1: Initialization Failed.\n\r");  
}

void loop() {
  delay(100);
  int vels[] = {500, 500, 500, 500};
  bool ext = true;
  for (int k = 0; k < 4; k++) {
    int vel = vels[k];
    mot = k;
    for (int m = 0; m < len; m++) {
      cTxData1[len - m -1] = (int)vel >> 8 * m;
    }
    int idd = mot | CAN_PACKET_SET_RPM << 8;  // 6/22/2021, added +1 because I indexed at 1, not 0, 4/9/2023, switched back, because of zero index
    canTx(1, idd, ext, cTxData1, len);
    delay(10);
  }
}
