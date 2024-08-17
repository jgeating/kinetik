#include <FlexCAN_T4.h>
#include "ODrive.h"
// #include <math.h>
// #include "utils.h"
#include "SwerveTelemetry.h"

// temp 
unsigned char steer_stmp[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
CAN_message_t steer_msg;

// CAN related
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

int id_steer = 0;       // Motor ID
int id_drive = 1;       // Drive motor ID
float pos = 0.0;        // Steering motor position
float steer_tff = 0.0;  // Steering motor feedforward torque

float vel = 0.0;  // Drive motor velocity
float drive_tff = 0.0;

ODrive steerMotor{ Can0, id_steer };
ODrive driveMotor{ Can0, id_drive };

SwerveTelemetry swerveTelemetry;

void setup(void) {
  Serial.begin(115200);
  // swerveTelemetry.start(); // requires ethernet to be ready

  delay(400);
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW); /* optional tranceiver enable pin */
  Can0.begin();
  Can0.setBaudRate(1000000);

  // Initialize Steering motor
  steerMotor.enablePrintOnWrite();
  steerMotor.setAbsolutePosition(0);
  delay(100);
  steerMotor.enableWithClosedLoop();
  // steerMotor.setVelocityControlMode();

  // Initialize drive motor
  driveMotor.enablePrintOnWrite();
  driveMotor.setAbsolutePosition(0);
  delay(100);
  driveMotor.enableWithClosedLoop();
  driveMotor.setVelocityControlMode();
  delay(100);
}

void loop() {

  // ***** Steering *****
  pos = pos + 0.001;
  Serial.print("Steering position: ");
  Serial.println(pos);
  steerMotor.setPosition(pos);
  // steerMotor.setVelocity(.25);
  delayMicroseconds(50);
  driveMotor.setVelocity(1);
  delayMicroseconds(50);
  // float position = driveMotor.getEncoderPosition();
  // float velocity = driveMotor.getEncoderVelocity();
  // Serial.print("Encoder Position: ");
  // Serial.println(position);
  // Serial.print("Encoder Velocity: ");
  // Serial.println(velocity);
  // delayMicroseconds(50);

  // swerveTelemetry.sendEncoderData(position, velocity);

  // Set loop speed roughly
  delay(5);
}
