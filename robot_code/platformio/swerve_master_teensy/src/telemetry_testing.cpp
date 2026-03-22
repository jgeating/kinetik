#include <Arduino.h>
#include "Telemetry.h"

// Telemetry
Telemetry swerveTelemetry;

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("Telemetry Testing");

  swerveTelemetry.start();
}

int num = 0;

void loop()
{
  num++;
  Serial.println(num);
  swerveTelemetry.sendInt("someString", 3);
  swerveTelemetry.send();
}
