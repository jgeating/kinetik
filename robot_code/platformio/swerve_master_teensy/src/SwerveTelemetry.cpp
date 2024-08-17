#include "SwerveTelemetry.h"

SwerveTelemetry::SwerveTelemetry() {
  m_ip = IPAddress(192, 168, 0, 6);
  m_remoteIp = IPAddress(192, 168, 0, 5);
}

void SwerveTelemetry::start() {
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

void SwerveTelemetry::sendPadData(double x, double y, double z) {
  memcpy(m_buffer, &x, 8);
  memcpy(m_buffer + 8, &y, 8);
  memcpy(m_buffer + 16, &z, 8);

  m_udp.beginPacket(m_remoteIp, m_udpPort);
  m_udp.write(m_buffer, 8);
  m_udp.endPacket();
}