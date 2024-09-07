#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

unsigned char stmp[8] = {0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int32_t ang = 0;
double osc_ang = 1000;   // oscillation angle, degrees
double ratio = 1;
CAN_message_t msg;
float pos = 0.0;  // Steering motor position

void setup(void) {
  Serial.begin(115200);
  delay(400);
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW); /* optional tranceiver enable pin */
  Can0.begin();
  Can0.setBaudRate(1000000);

  Serial.println("Sending motor to starting position");
  ang = -osc_ang * 100 * ratio / 2;
  stmp[4] = *((uint8_t *)(&ang));
  stmp[5] = *((uint8_t *)(&ang)+1);
  stmp[6] = *((uint8_t *)(&ang)+2);
  stmp[7] = *((uint8_t *)(&ang)+3);
  stmp[0] = 0xA3;
  stmp[3] = 0x03;
  // canTx(0, 0x141, false, stmp, 8);
  // stmp[0] = 0xA3;
  // stmp[3] = 0x00;

  msg.id = 0x141;
  msg.flags.extended = false;
  for ( uint8_t i = 0; i < 8; i++ ){
    msg.buf[i] = stmp[i];
  }
  Can0.write(msg);
  delay(2000);
}

void loop() {
  // ang = ang + 10;
  // Serial.print("Steering position: ");
  // Serial.println(ang);
  // memcpy(msg.buf, &ang, sizeof(ang));
  // msg.id = 0x141;
  // msg.len = sizeof(stmp);
  // for ( uint8_t i = 0; i < 8; i++ ){
  //   Serial.print(msg.buf[i], HEX);
  //   if (i < 7){
  //     Serial.print(", ");
  //   } else {
  //     Serial.println();
  //   }
  // } 
  // Can0.write(msg);
  // delayMicroseconds(50);
  // delay(100);
}
