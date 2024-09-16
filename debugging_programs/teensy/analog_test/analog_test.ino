#include <FlexCAN_T4.h>
#include "Pads.h"

int apins[] = { A15, A14, A17, A16, A1, A0, A3, A2 };
int offsets[] = { 115, 153, 176, 154, 193, 36, 173, 286 };
int npins = 8;
int can_len = 6;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> canBus;
CAN_message_t canTxMsg;
CAN_message_t canRxMsg;

Pads *pads;  // For driving with force pads

int count = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  canBus.begin();
  canBus.setBaudRate(1000000);

  canTxMsg.id = 10;
  canTxMsg.len = can_len;
  canTxMsg.flags.extended = false;

  pads = new Pads();
  pads->calibrate();
}

void printPins() {
  // Serial.println("******");
  for (int k = 0; k < npins; k++) {
    // for (int k = 3; k < 4; k++){
    // int k = 1;
    // Serial.print(k);
    Serial.print((int)(analogRead(apins[k])) - offsets[k]);
    if (k < npins - 1) {
      Serial.print(",");
    }
    // Serial.println((int)(analogRead(apins[k])) - offsets[k]);
  }
  Serial.println();
}

void loop() {
  // printPins();
  // count++;

  pads->calcVector();

  double xRaw = pads->getX();
  double yRaw = pads->getY();
  double zRaw = pads->getZ();

  // for (int i = 0; i < 8; i++){
  //   Serial.print(pads->getForce(i));
  //   if (i < 7){
  //     Serial.print(", ");
  //   } else {
  //     Serial.println();
  //   }
  // }
  // Serial.print(", ");
  // Serial.print(yRaw);
  // Serial.print(", ");
  // Serial.println(zRaw);
  

  uint16_t x = (xRaw + 1) * pow(2, 15);
  uint16_t y = (yRaw + 1) * pow(2, 15);
  uint16_t z = (zRaw + 1) * pow(2, 15);

  memcpy(canTxMsg.buf, &x, 2);
  memcpy(canTxMsg.buf + 2, &y, 2);
  memcpy(canTxMsg.buf + 4, &z, 2);

  // if (count % 20 == 0 || true) {
    canBus.write(canTxMsg);
    // Serial.println("********");
    // Serial.print("ID: ");
    // Serial.print(canRxMsg.id);
    // Serial.println();
    // Serial.print("ext: ");
    // Serial.println(canTxMsg.flags.extended);
    // Serial.print("len: ");
    // Serial.println(canTxMsg.len);
    delayMicroseconds(3000);  // Loop slightly slower than the due 
  // }
}
