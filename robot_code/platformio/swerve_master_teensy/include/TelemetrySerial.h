#ifndef __TELEMETRY_SERIAL_H
#define __TELEMETRY_SERIAL_H

#include <Arduino.h>
#include <ArduinoJson.h>

class TelemetrySerial {

public:
  // Pass any serial-like object (Serial, Serial1, Serial2, …).
  // The caller is responsible for calling Serial.begin() before start().
  TelemetrySerial(Stream& serial = Serial);
  void start();

  // Accumulate values into the next serial packet
  void sendInt(const String& name, int value);
  void sendFloat(const String& name, float value);
  void sendDouble(const String& name, double value);
  void sendBool(const String& name, bool value);
  void sendString(const String& name, const String& value);
  void sendIntArray(const String& name, int* value, size_t len);
  void sendFloatArray(const String& name, float* value, size_t len);
  void sendDoubleArray(const String& name, double* value, size_t len);
  void sendBoolArray(const String& name, bool* value, size_t len);

  // Serialize accumulated data and transmit over serial, then clear the buffer
  void send();

private:
  Stream& m_serial;
  JsonDocument m_doc;
  char m_txBuffer[2048];
};

/********************************************************************/
#endif  // __TELEMETRY_SERIAL_H
