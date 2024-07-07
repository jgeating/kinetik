/* This example shows how to use continuous mode to take
range measurements with the VL53L0X. It is based on
vl53l0x_ContinuousRanging_Example.c from the VL53L0X API.

The range readings are in units of mm. */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

const int numReadings = 35; // Number of readings to consider
float readings[numReadings]; // Array to store sensor readings
const float coefficient = .8; // Weighting coefficient


void setup()
{
  Serial.begin(115200);
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  sensor.startContinuous();

  // Initialize the array with zeros
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0.0;
  }
}

void loop() {
  // Read sensor value
  float sensorValue = sensor.readRangeContinuousMillimeters();
  // Compute weighted moving average
  float weightedMovingAverage = computeWeightedMovingAverage(sensorValue);
  // Print the result
  Serial.println(weightedMovingAverage);
}

float computeWeightedMovingAverage(float newValue) {
  static float prevWeightedMovingAverage = 0.0;

  // Shift all the elements of the array
  for (int i = numReadings - 1; i > 0; i--) {
    readings[i] = readings[i - 1];
  }

  // Insert the new value into the array
  readings[0] = newValue;

  // Compute the weighted moving average
  float weightedMovingAverage = 0.0;
  for (int i = 0; i < numReadings; i++) {
    weightedMovingAverage += readings[i] * pow(coefficient, i);
  }

  prevWeightedMovingAverage = weightedMovingAverage;
  return prevWeightedMovingAverage;
}

