#include "Swerve.h"
#include "Arduino.h"

void printWatchdogError(Watchdog &watchdog)
{
  unsigned long time = micros();

  if (time - watchdog.prevReportTime > watchdog.timeBetweenReportsMicros)
  {

    if (time - watchdog.prevLoopTime > watchdog.loopTimeMicros)
    {
      serialPrintln(100, "Error! Loop time overrun! Time: %d", time - watchdog.prevLoopTime);
    }

    watchdog.prevReportTime = time;
  }

  watchdog.prevLoopTime = time;
}

void serialPrintln(int bufferSize, const char *format, ...)
{
  char buffer[bufferSize];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, bufferSize, format, args);
  va_end(args);
  Serial.println(buffer);
}

void serialPrint(int bufferSize, const char *format, ...)
{
  char buffer[bufferSize];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, bufferSize, format, args);
  va_end(args);
  Serial.print(buffer);
}