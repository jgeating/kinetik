#include <FlexCAN_T4.h>
// #include "ODrive.h"
#include "Vesc.h"
#include "sbus.h"
#include "SbusReceiver.h"

// temp
unsigned char steer_stmp[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
CAN_message_t steer_msg;

// CAN related
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

int id_steer = 0;       // Motor ID
int id_drive = 10;       // Drive motor ID
float pos = 0.0;        // Steering motor position
float steer_tff = 0.0;  // Steering motor feedforward torque

double vel = 0.0;  // Drive motor velocity
float drive_tff = 0.0;

// ODrive steerMotor{ Can0, id_steer };
Vesc driveMotor{ Can0, id_drive };
SbusReceiver sbusReceiver;

void setup(void) {
  Serial.begin(115200);
  Serial.print("Setting up...");

  delay(400);
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW); /* optional tranceiver enable pin */
  Can0.begin();
  Can0.setBaudRate(1000000);

  // Initialize Steering motor
  // steerMotor.enablePrintOnWrite();
  // steerMotor.setAbsolutePosition(0);
  delay(100);
  // steerMotor.enableWithClosedLoop();

  // Initialize drive motor
  driveMotor.enablePrintOnWrite();
  driveMotor.setAbsolutePosition(0);
  delay(100);
  driveMotor.enableWithClosedLoop();
  driveMotor.setVelocityControlMode();
  delay(100);

  sbusReceiver.init();

  delay(3000);

}

void loop() {
  sbusReceiver.read();
  // steerMotor.setPosition(pos);
  vel += .05;
  delayMicroseconds(50);
  Serial.print("RIGHT Y: ");
  Serial.print(sbusReceiver.getRightVert());
  Serial.println();
  Serial.println("CAN msg:");
  // driveMotor.setVelocity(fmod(vel, 10));
  driveMotor.setVelocity(10 * sbusReceiver.getLeftVert());

  delayMicroseconds(50);

  float position = driveMotor.getEncoderPosition();
  float velocity = driveMotor.getEncoderVelocity();
  
  delay(5);
}
