#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

unsigned char stmp[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
int32_t ang = 0;
CAN_message_t msg;
int mot = 0;

void setPosition(uint16_t speedLimit, int32_t angle) {
  speedLimit *= 18;
  angle *= 100 * 18;
  msg.buf[0] = 0xA4; // Position control command 2
  msg.buf[1] = 0;
  memcpy(msg.buf + 2, &speedLimit, 2);
  memcpy(msg.buf + 4, &angle, 4);
  Can0.write(msg);
}

void setup(void) {
  Serial.begin(115200);
  delay(400);
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW); /* optional tranceiver enable pin */
  Can0.begin();
  Can0.setBaudRate(1000000);

  digitalWrite(13, HIGH);

  msg.id = 0x141 + mot;
  msg.flags.extended = false;
  delay(1000);
  setPosition(100, 0);
  delay(5000);
}

void loop() {
  ang += 180;
  setPosition(360, ang);
  Serial.printf("Angle: %d\n", ang);
  delay(2000);
}
