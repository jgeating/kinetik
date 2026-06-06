#include "Telemetry.h"

Telemetry::Telemetry() {
  m_ip = IPAddress(TEENSY_IP_ADDR);
  m_remoteIp = IPAddress(RECEIVER_IP_ADDR);
}

void Telemetry::start() {
  // If the configured static IP is 0.0.0.0, use DHCP; otherwise use the static IP.
  bool useDhcp = (m_ip == IPAddress(0, 0, 0, 0));
  if (useDhcp) {
    Serial.println("[Telemetry] Requesting IP via DHCP...");
    // DHCP can take a few seconds; give it a bounded timeout.
    if (Ethernet.begin(m_mac, 8000, 4000) == 0) {
      Serial.println("[Telemetry] WARNING: DHCP failed (no lease).");
    }
  } else {
    Ethernet.begin(m_mac, m_ip);
  }

  // Non-blocking startup: don't wait for Serial
  // Check for Ethernet hardware present (with timeout)
  unsigned long startTime = millis();
  while (!Ethernet.localIP() && (millis() - startTime) < 1000) {
    delay(100);  // Brief delay to allow Ethernet to initialize
  }

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("[Telemetry] ERROR: Ethernet hardware not found. Check RJ45 shield.");
    m_ethernetReady = false;
    return;
  }

  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("[Telemetry] ERROR: Ethernet cable not connected.");
    m_ethernetReady = false;
    return;
  }

  // Start UDP only if Ethernet is actually ready
  if (Ethernet.localIP() != IPAddress(0, 0, 0, 0)) {
    m_udp.begin(m_udpPort);
    m_ethernetReady = true;
    Serial.print("[Telemetry] SUCCESS: Ethernet initialized. Teensy IP: ");
    Serial.println(Ethernet.localIP());
    Serial.print("[Telemetry] Sending telemetry to ");
    Serial.print(m_remoteIp);
    Serial.print(":");
    Serial.println(m_udpPort);
  } else {
    Serial.println("[Telemetry] ERROR: Failed to obtain IP address.");
    m_ethernetReady = false;
  }
}

void Telemetry::retryIfNeeded() {
  if (m_ethernetReady) {
    return;  // Already up; nothing to do.
  }

  // Throttle retries so we don't block the loop too often.
  static unsigned long lastRetry = 0;
  if (millis() - lastRetry < 5000) {
    return;
  }
  lastRetry = millis();

  // Need a live link before DHCP can succeed.
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("[Telemetry] retry: link is OFF, waiting...");
    return;
  }

  bool useDhcp = (m_ip == IPAddress(0, 0, 0, 0));
  if (useDhcp) {
    Serial.println("[Telemetry] retry: requesting DHCP lease...");
    if (Ethernet.begin(m_mac, 8000, 4000) == 0) {
      Serial.println("[Telemetry] retry: DHCP still failing.");
      return;
    }
  } else {
    Ethernet.begin(m_mac, m_ip);
  }

  if (Ethernet.localIP() != IPAddress(0, 0, 0, 0)) {
    m_udp.begin(m_udpPort);
    m_ethernetReady = true;
    Serial.print("[Telemetry] retry SUCCESS: Teensy IP: ");
    Serial.println(Ethernet.localIP());
  }
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
  if (!m_ethernetReady) {
    m_doc.clear();  // Clear without sending
    return;
  }

  size_t n = serializeJson(m_doc, m_txBuffer, sizeof(m_txBuffer));

  // Destination: if RECEIVER_IP is unset (0.0.0.0) OR is on a different subnet
  // than our DHCP-assigned IP, fall back to the subnet broadcast address so any
  // client on this network bound to the UDP port receives telemetry. This avoids
  // needing to know the client's exact IP ahead of time.
  IPAddress dest = m_remoteIp;
  IPAddress local = Ethernet.localIP();
  IPAddress mask = Ethernet.subnetMask();
  bool remoteUnset = (m_remoteIp == IPAddress(0, 0, 0, 0));
  bool sameSubnet = true;
  for (int i = 0; i < 4; i++) {
    if ((local[i] & mask[i]) != (m_remoteIp[i] & mask[i])) {
      sameSubnet = false;
    }
  }
  if (remoteUnset || !sameSubnet) {
    for (int i = 0; i < 4; i++) {
      dest[i] = (local[i] & mask[i]) | (~mask[i] & 0xFF);  // subnet broadcast
    }
  }

  m_udp.beginPacket(dest, m_udpPort);
  m_udp.write((uint8_t*)m_txBuffer, n);
  m_udp.endPacket();
  m_doc.clear();
}

void Telemetry::sendToSerial() {
  if (!m_doc.size()) {
    return;  // Nothing to send
  }

  size_t n = serializeJson(m_doc, m_txBuffer, sizeof(m_txBuffer));
  Serial.print("[TELEM] ");
  Serial.write((uint8_t*)m_txBuffer, n);
  Serial.println();
  m_doc.clear();
}