#include <FlexCAN_T4.h>
#include "ODrive.h"
// #include <math.h>
// #include "utils.h"

// CAN related
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

int id_steer = 1;       // Motor ID
int id_drive = 0;       // Drive motor ID
float pos = 0.0;        // Steering motor position
float steer_tff = 0.0;  // Steering motor feedforward torque

float vel = 0.0;  // Drive motor velocity
float drive_tff = 0.0;

ODrive steerMotor{ Can0, id_steer };
ODrive driveMotor{ Can0, id_drive };

void setup(void) {
  Serial.begin(115200);
  delay(400);
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW); /* optional tranceiver enable pin */
  Can0.begin();
  Can0.setBaudRate(1000000);

  // Initialize ODrive
  steerMotor.enablePrintOnWrite();
  steerMotor.setAbsolutePosition(0);
  driveMotor.enablePrintOnWrite();
  driveMotor.setAbsolutePosition(0);
  delay(3000);
  steerMotor.enableWithClosedLoop();
  steerMotor.setVelocityControlMode();
  driveMotor.enableWithClosedLoop();
  driveMotor.setVelocityControlMode();
  delay(1000);
}

void loop() {

  // ***** Steering *****
  // pos = pos - 0.001;
  // Serial.print("Steering position: ");
  // Serial.println(pos);
  // steerMotor.setPosition(pos);
  steerMotor.setVelocity(.25);
  delayMicroseconds(50);
  driveMotor.setVelocity(3);
  delayMicroseconds(50);
  Serial.print("Encoder Position: ");
  Serial.println(steerMotor.getEncoderPosition());
  Serial.print("Encoder Velocity: ");
  Serial.println(steerMotor.getEncoderVelocity());
  delayMicroseconds(50);

  // Set loop speed roughly
  delay(5);
}
