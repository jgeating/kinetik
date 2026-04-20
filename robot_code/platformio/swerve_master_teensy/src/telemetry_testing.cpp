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

  swerveTelemetry.sendBool("someBool", false);

}

int num = 0;

void loop()
{
  num++;
  Serial.println(num);
  swerveTelemetry.sendBool("someBool", false);
  swerveTelemetry.sendInt("someString", num);
  swerveTelemetry.send();
}
