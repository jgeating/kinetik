#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"

//   Connections
//   ===========
//   Connect SCL to analog 5
//   Connect SDA to analog 4
//   Connect VDD to 3.3V DC
//   Connect GROUND to common ground

#define BNO055_SAMPLERATE_DELAY_MS (10)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
}

void loop() {
  // Geometry variables
//  Rotation R0, R1;  // R0 = raw sensor data, R1 = rider frames
//  Point p1, p2;
//  p1.Y() = 4.27;
//  p2.X() = -9.12;
//  p2.Z() = 3.32;


  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // You can also populate a rotation matrix using euler angles (roll, pitch, yaw) which is a good way to work with gyro sensor data without having to worry about gimbal lock
//  R0.FromEulerAngles(euler.x(), euler.y(), euler.z());

  // You can rotate Points by premultiplying them with a Rotation like so:
//  Point p3 = R1 * p1;

  // The points will be rotated to different coordinates
//  Serial << "p1 = " << p1 << "\np3 = " << p3 << "\n";

  // Likewise you can rotate rotations by multiplying them together. Formally speaking, to rotate R0 by a rotation represented by R1 you must premultiply R0 by R1 like so:
//  Rotation R4 = R1 * R0;

  // Rotation matrices have the property that their transpose is their inverse so we can undo a rotation by premultiplying by the transpose. So, the following statement rotates R1, then rotates it back again:
//  Serial << "R0 =             " << R0 << "\nR1^T * R1 * R0 = " << (~R1 * R1 * R0) << "\n\n";

    /* Display calibration status for each sensor. */
/*  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
*/  
  delay(BNO055_SAMPLERATE_DELAY_MS);
  }
