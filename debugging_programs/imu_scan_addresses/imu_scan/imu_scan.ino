#include "DueCANLayer.h"
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"

#define BNO055_SAMPLERATE_DELAY_MS (10)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno; 

void setup() {
  // Setup serial
  Serial.begin(115200);
  Serial.println("Starting up...");
  pinMode(13, OUTPUT);
  //analogReadResolution(anRes);

  for (int i = 100; i < 256; i++){
    Adafruit_BNO055(-1, i);
    if(!bno.begin())
    {
      Serial.print(", ");
      Serial.print(i);
    } else {
      Serial.print("Connection on: ");
      Serial.println(i);
      break;
    }
    delay(100);
  }

    bno.setExtCrystalUse(true);


  delay(1000);    // Give motor time to move
}

void loop() {
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  // check for IMU request from master
  // Check for received message
  //if(canRx(0, &lMsgID, &bExtendedFormat, &cRxData[0], &cDataLen) == CAN_OK)
  //{
    // if request was master requesting IMU data (0xA7)
    //if(lMsgID == canID && cRxData[0] == 0xA7)
    //{
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      
      /* Display the floating point data */

      /*
      Serial.print("X: ");
      Serial.print(euler.x());
      Serial.print(" Y: ");
      Serial.print(euler.y());
      Serial.print(" Z: ");
      Serial.println(euler.z());
      delay(20);
      */
    //}
  //}
}
