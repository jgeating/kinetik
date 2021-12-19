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

// Currently, a Teensy 3.X is connected

#define BNO055_SAMPLERATE_DELAY_MS (10)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
imu::Vector<3> euler;
double ytemp, ztemp;
int yint, zint;
void setup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  analogWriteResolution(12);

  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);

  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(14, OUTPUT);
  pinMode(16, OUTPUT);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);
}

void loop(void)
{
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

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
  ytemp = euler.y();
  ztemp = euler.z();
  if (ytemp < 0){
    ytemp = ytemp + 360;
  }
  yint = round(map(ytemp, 0, 360, 205, 3981));  // use 5-95% duty cycle to ensure interrupts (no continuous high or low)
  if (ztemp < 0){
    ztemp = ztemp + 360;
  }
  zint = round(map(ztemp, 0, 360, 205, 3981));
  Serial.print(" Y: ");
  //Serial.print(yint);
  Serial.print(pulseIn(1, 20000));
  Serial.print(" Z: ");
  Serial.print(pulseIn(2, 20000));
  //Serial.print(zint);
  Serial.print("\t\t");
  Serial.println();
  
  analogWrite(14, yint);
  analogWrite(16, zint);
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
