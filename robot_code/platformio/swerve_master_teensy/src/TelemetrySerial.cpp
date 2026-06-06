#include "TelemetrySerial.h"

TelemetrySerial::TelemetrySerial(Stream& serial)
    : m_serial(serial) {}

void TelemetrySerial::start() {
  // Serial.begin() should be called by the caller before start().
  // Wait here until the port is ready (relevant for USB virtual serial).
  while (!Serial) {
    ;
  }
}

void TelemetrySerial::sendInt(const String& name, int value) {
  m_doc[name] = value;
}

void TelemetrySerial::sendFloat(const String& name, float value) {
  m_doc[name] = value;
}

void TelemetrySerial::sendDouble(const String& name, double value) {
  m_doc[name] = value;
}

void TelemetrySerial::sendBool(const String& name, bool value) {
  m_doc[name] = value;
}

void TelemetrySerial::sendString(const String& name, const String& value) {
  m_doc[name] = value;
}

void TelemetrySerial::sendIntArray(const String& name, int* value, size_t len) {
  JsonArray arr = m_doc[name].to<JsonArray>();
  for (size_t i = 0; i < len; i++) {
    arr.add(value[i]);
  }
}

void TelemetrySerial::sendFloatArray(const String& name, float* value, size_t len) {
  JsonArray arr = m_doc[name].to<JsonArray>();
  for (size_t i = 0; i < len; i++) {
    arr.add(value[i]);
  }
}

void TelemetrySerial::sendDoubleArray(const String& name, double* value, size_t len) {
  JsonArray arr = m_doc[name].to<JsonArray>();
  for (size_t i = 0; i < len; i++) {
    arr.add(value[i]);
  }
}

void TelemetrySerial::sendBoolArray(const String& name, bool* value, size_t len) {
  JsonArray arr = m_doc[name].to<JsonArray>();
  for (size_t i = 0; i < len; i++) {
    arr.add(value[i]);
  }
}

void TelemetrySerial::send() {
  size_t n = serializeJson(m_doc, m_txBuffer, sizeof(m_txBuffer));
  m_serial.write((uint8_t*)m_txBuffer, n);
  m_serial.write('\n');  // newline delimiter so the receiver can frame packets
  m_doc.clear();
}
