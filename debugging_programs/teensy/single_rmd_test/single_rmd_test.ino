#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

unsigned char stmp[8] = {0xA3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int32_t ang = 0;
double osc_ang = 1800;   // oscillation angle, degrees
double ratio = 1;
CAN_message_t msg;
 
void setup(void) {
  Serial.begin(115200); delay(400);
  pinMode(6, OUTPUT); digitalWrite(6, LOW); /* optional tranceiver enable pin */
  Can0.begin();
  Can0.setBaudRate(1000000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(canSniff);
  Can0.mailboxStatus();


  Serial.println("Sending motor to starting position");
  ang = -osc_ang * 100 * ratio / 2;
  stmp[4] = *((uint8_t *)(&ang));
  stmp[5] = *((uint8_t *)(&ang)+1);
  stmp[6] = *((uint8_t *)(&ang)+2);
  stmp[7] = *((uint8_t *)(&ang)+3);
  stmp[0] = 0xA4;
  stmp[3] = 0x03;
  // canTx(0, 0x141, false, stmp, 8);
  stmp[0] = 0xA3;
  stmp[3] = 0x00;

  msg.id = 0x141;
  msg.flags.extended = false;
  for ( uint8_t i = 0; i < 8; i++ ){
    msg.buf[i] = stmp[i];
  }
  Can0.write(msg);
  delay(2000);
}

void canSniff(const CAN_message_t &msg) {
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}

void loop() {
  Can0.events();

  static uint32_t timeout = millis();
  if ( millis() - timeout > 200 ) {
    CAN_message_t msg;
    msg.id = random(0x1,0x7FE);
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = i + 1;
    // Can0.write(msg);
    timeout = millis();
  }

}
