#include "Telemetry.h"

Telemetry::Telemetry() {
  m_ip = IPAddress(192, 168, 0, 6);
  m_remoteIp = IPAddress(192, 168, 0, 5);
}

void Telemetry::start() {
  Ethernet.begin(m_mac, m_ip);

  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1);  // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  // start UDP
  m_udp.begin(m_udpPort);
}

void Telemetry::sendInt(const String& name, int value) {
  m_doc[name] = value;
}

void Telemetry::sendFloat(const String& name, float value) {
  m_doc[name] = value;
}

void Telemetry::sendDouble(const String& name, double value) {
  m_doc[name] = value;
}

void Telemetry::sendBool(const String& name, bool value) {
  m_doc[name] = value;
}

void Telemetry::sendString(const String& name, const String& value) {
  m_doc[name] = value;
}

void Telemetry::sendIntArray(const String& name, int* value, size_t len) {
  JsonArray arr = m_doc[name].to<JsonArray>();
  for (size_t i = 0; i < len; i++) {
    arr.add(value[i]);
  }
}

void Telemetry::sendFloatArray(const String& name, float* value, size_t len) {
  JsonArray arr = m_doc[name].to<JsonArray>();
  for (size_t i = 0; i < len; i++) {
    arr.add(value[i]);
  }
}

void Telemetry::sendDoubleArray(const String& name, double* value, size_t len) {
  JsonArray arr = m_doc[name].to<JsonArray>();
  for (size_t i = 0; i < len; i++) {
    arr.add(value[i]);
  }
}

void Telemetry::sendBoolArray(const String& name, bool* value, size_t len) {
  JsonArray arr = m_doc[name].to<JsonArray>();
  for (size_t i = 0; i < len; i++) {
    arr.add(value[i]);
  }
}

void Telemetry::send() {
  size_t n = serializeJson(m_doc, m_txBuffer, sizeof(m_txBuffer));
  m_udp.beginPacket(m_remoteIp, m_udpPort);
  m_udp.write((uint8_t*)m_txBuffer, n);
  m_udp.endPacket();
  m_doc.clear();
}