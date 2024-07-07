#include <FlexCAN_T4.h>
// #include <math.h>
// #include "utils.h"

// CAN related
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

unsigned char steer_stmp[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char drive_stmp[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
bool ext = false;   // Extended format flag
int id_steer = 1;       // Motor ID 
int id_drive = 0;       // Drive motor ID
CAN_message_t steer_msg;
CAN_message_t drive_msg;
float pos = 0.0;  // Steering motor position
float steer_tff = 0.0;  // Steering motor feedforward torque 

float vel = 0.0;  // Drive motor velocity 
float drive_tff = 0.0;

void setup(void) {
  Serial.begin(115200); delay(400);
  pinMode(6, OUTPUT); digitalWrite(6, LOW); /* optional tranceiver enable pin */
  Can0.begin();
  Can0.setBaudRate(1000000);

  // Initialize ODrive 
  steer_msg.id = id_steer << 5 | 0x19;  // reset position to 0 (if odrive has been on)
  steer_msg.flags.extended = false;
  // steer_msg.buf[0] = 0x00;
  // steer_msg.buf[1] = 0x00;
  // steer_msg.buf[2] = 0x00;
  // steer_msg.buf[3] = 0x00;
  steer_msg.len = 8;

  Can0.write(steer_msg);
  delay(3000);
}

void loop() {

  // ***** Steering *****
  pos = pos + 0.001;
  Serial.print("Steering position: ");
  Serial.println(pos);
  memcpy(steer_stmp, &pos, sizeof(pos));
  memcpy(steer_stmp + sizeof(pos), &steer_tff, sizeof(steer_tff));
  steer_msg.id = id_steer << 5 | 0x0c;
  steer_msg.flags.extended = false;
  steer_msg.len = sizeof(steer_stmp);
  for ( uint8_t i = 0; i < 8; i++ ){
    steer_msg.buf[i] = steer_stmp[i];
    Serial.print(steer_msg.buf[i], HEX);
    if (i < 7){
      Serial.print(", ");
    } else {
      Serial.println();
    }
  } 
  Can0.write(steer_msg);
  delayMicroseconds(50);

  // ***** Drive *****
  vel = vel + .001;
  Serial.print("drive vel: ");
  Serial.println(vel);
  memcpy(drive_stmp, &vel, sizeof(vel));
  memcpy(drive_stmp + sizeof(vel), &drive_tff, sizeof(drive_stmp));
  drive_msg.id = id_drive << 5 | 0x0c;
  drive_msg.flags.extended = false;
  drive_msg.len = sizeof(drive_stmp);
  for ( uint8_t i = 0; i < 8; i++ ){
    drive_msg.buf[i] = drive_stmp[i];
    Serial.print(drive_msg.buf[i], HEX);
    if (i < 7){
      Serial.print(", ");
    } else {
      Serial.println();
    }
  } 
  Can0.write(drive_msg);
  delayMicroseconds(50);

  // Set loop speed roughly 
  delay(5);
}
