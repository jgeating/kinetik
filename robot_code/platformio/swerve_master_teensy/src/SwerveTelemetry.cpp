#include "SwerveTelemetry.h"

SwerveTelemetry::SwerveTelemetry() {
  m_ip = IPAddress(192, 168, 0, 6);
  m_remoteIp = IPAddress(192, 168, 0, 5);
}

void SwerveTelemetry::start() {
  Ethernet.begin(m_mac, m_ip);

  // Non-blocking startup: don't wait for Serial
  // Check for Ethernet hardware present (with timeout)
  unsigned long startTime = millis();
  while (!Ethernet.localIP() && (millis() - startTime) < 1000) {
    delay(100);  // Brief delay to allow Ethernet to initialize
  }

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("[Telemetry] WARNING: Ethernet hardware not found. Continuing without telemetry.");
    m_ethernetReady = false;
    return;
  }

  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("[Telemetry] WARNING: Ethernet cable not connected. Continuing without telemetry.");
    m_ethernetReady = false;
    return;
  }

  // Start UDP only if Ethernet is actually ready
  if (Ethernet.localIP() != IPAddress(0, 0, 0, 0)) {
    m_udp.begin(m_udpPort);
    m_ethernetReady = true;
    Serial.print("[Telemetry] Ready at ");
    Serial.println(Ethernet.localIP());
  } else {
    Serial.println("[Telemetry] WARNING: Failed to obtain IP. Continuing without telemetry.");
    m_ethernetReady = false;
  }
}

void SwerveTelemetry::sendPadData(double x, double y, double z) {
  memcpy(m_buffer, &x, 8);
  memcpy(m_buffer + 8, &y, 8);
  memcpy(m_buffer + 16, &z, 8);

  m_udp.beginPacket(m_remoteIp, m_udpPort);
  m_udp.write(m_buffer, 8);
  m_udp.endPacket();
}

void SwerveTelemetry::queueDouble(double value) {
  if (!m_ethernetReady) return;  // Silent no-op if Ethernet unavailable
  
  if (m_bufferIndex < BUFFER_SIZE) {
    m_doubleBuffer[m_bufferIndex++] = value;
  } else {
    // Buffer full; send immediately
    sendBufferedData();
    m_doubleBuffer[0] = value;
    m_bufferIndex = 1;
  }
}

void SwerveTelemetry::sendBufferedData() {
  if (!m_ethernetReady || m_bufferIndex == 0) return;
  
  m_udp.beginPacket(m_remoteIp, m_udpPort);
  m_udp.write((uint8_t*)m_doubleBuffer, m_bufferIndex * sizeof(double));
  m_udp.endPacket();
  m_bufferIndex = 0;
}