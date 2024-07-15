#ifndef __SWERVE_TELEMETRY_H
#define __SWERVE_TELEMETRY_H

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

static void teensyMAC(byte* mac) {
  for (uint8_t by = 0; by < 2; by++) mac[by] = (HW_OCOTP_MAC1 >> ((1 - by) * 8)) & 0xFF;
  for (uint8_t by = 0; by < 4; by++) mac[by + 2] = (HW_OCOTP_MAC0 >> ((3 - by) * 8)) & 0xFF;
  Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

class SwerveTelemetry {

public:
  SwerveTelemetry();
  void start();
  void sendEncoderData(float position, float velocity);

private:
  byte m_mac[6] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
  };
  IPAddress m_ip;
  IPAddress m_remoteIp;
  unsigned int m_udpPort = 8888;  // local port to listen on
  char m_buffer[64];
  uint16_t m_data1 = 5;
  uint8_t m_data2 = 10;
  double m_data3 = 10.5;
  EthernetUDP m_udp;
};

/********************************************************************/
#endif  // __SWERVE_TELEMETRY_H