#ifndef __TELEMETRY_H
#define __TELEMETRY_H

#include <Ethernet.h>
#include <EthernetUdp.h>
#include <ArduinoJson.h>

class Telemetry {

public:
  Telemetry();
  void start();

  // Accumulate values into the next UDP packet
  void sendInt(const String& name, int value);
  void sendFloat(const String& name, float value);
  void sendDouble(const String& name, double value);
  void sendBool(const String& name, bool value);
  void sendString(const String& name, const String& value);
  void sendIntArray(const String& name, int* value, size_t len);
  void sendFloatArray(const String& name, float* value, size_t len);
  void sendDoubleArray(const String& name, double* value, size_t len);
  void sendBoolArray(const String& name, bool* value, size_t len);

  // Serialize accumulated data and transmit over UDP, then clear the buffer
  void send();

private:
  byte m_mac[6] = {0xA8, 0x61, 0x0A, 0xAE, 0x00, 0xB9};  // Arduino shield mac address
  // byte m_mac[6] = {0xCC, 0xBA, 0xBD, 0xCB, 0x3B, 0x7C};
  IPAddress m_ip;
  IPAddress m_remoteIp;
  unsigned int m_udpPort = 8888;
  EthernetUDP m_udp;
  JsonDocument m_doc;
  char m_txBuffer[512];
};

/********************************************************************/
#endif  // __TELEMETRY_H