#ifndef __TELEMETRY_H
#define __TELEMETRY_H

#include <NativeEthernet.h>
#include <EthernetUdp.h>
#include <ArduinoJson.h>
#include "NetworkConfig.h"

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

  // Serialize accumulated data and transmit over UDP or Serial, then clear the buffer
  void send();
  
  // Send telemetry directly to Serial (USB) instead of Ethernet/UDP
  void sendToSerial();

  // Diagnostics
  bool isReady() const { return m_ethernetReady; }
  IPAddress localIp() const { return Ethernet.localIP(); }
  IPAddress remoteIp() const { return m_remoteIp; }
  unsigned int udpPort() const { return m_udpPort; }
  int linkStatus() const { return (int)Ethernet.linkStatus(); }  // 0=Unknown,1=On,2=Off

  // If not yet ready (e.g. DHCP not acquired or link was down at boot),
  // periodically retry bringing up Ethernet. Call from the slow/outer loop.
  void retryIfNeeded();

private:
  byte m_mac[6] = {0xA8, 0x61, 0x0A, 0xAE, 0x00, 0xB9};  // Arduino shield mac address
  // byte m_mac[6] = {0xCC, 0xBA, 0xBD, 0xCB, 0x3B, 0x7C};
  IPAddress m_ip;
  IPAddress m_remoteIp;
  unsigned int m_udpPort = 8888;
  EthernetUDP m_udp;
  JsonDocument m_doc;
  char m_txBuffer[512];
  bool m_ethernetReady = false;
};

/********************************************************************/
#endif  // __TELEMETRY_H