#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"
#include "Geometry.h"

//   Connections
//   ===========
//   Connect SCL to analog 5
//   Connect SDA to analog 4
//   Connect VDD to 3.3V DC
//   Connect GROUND to common ground

#define BNO055_SAMPLERATE_DELAY_MS (10)

// IMU Stuff
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

// Geometry Stuff
Rotation R0, R1, R0_1;  // R0 = raw sensor data, R1 = rider frames, R0_1 = rotation matrix from 0 to 1
Point p1, p2;
double y_start, z_start;
imu::Vector<3> euler;   // For outputs from bno

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(10);
  bno.setExtCrystalUse(true);
  Serial.println("delaying for calibration...");
  delay(3000);

  // Initialize and orient starting frame
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  y_start = euler.y();
  z_start = euler.z();
  Serial.printf("Start y: %.2f\n", y_start);
  Serial.printf("Start z: %.2f\n", z_start);
}

void loop(void)
{
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  R0.FromEulerAngles(euler.x()*3.141592/180, euler.y()*3.141592/180, euler.z()*3.141592/180);
  R1 = R0_1 * R0;
  //R1 = R0;
  Serial.printf("X: %.2f\n", euler.x());
  Serial.printf("Y: %.2f\n", euler.y() - y_start);
  Serial.printf("Z: %.2f\n", euler.z() - z_start);
  
  //Serial << R1.ToEulerAngles() << "\n\n";
  Matrix<3, 1> eul1 = R1.ToEulerAngles().Submatrix(Slice<0,3>(), Slice<0,1>());
  //Serial << "X: " << eul1(0) << '\n';
  //Serial << "Y: " << eul1(1) << '\n';
  //Serial << "Z: " << eul1(2) << '\n';

  /* Display the floating point data */
  /*
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\t\t");
  Serial.println();
  */
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
