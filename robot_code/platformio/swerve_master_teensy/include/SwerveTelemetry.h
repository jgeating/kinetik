#ifndef __SWERVE_TELEMETRY_H
#define __SWERVE_TELEMETRY_H

#include <Ethernet.h>
#include <EthernetUdp.h>

class SwerveTelemetry {

public:
  SwerveTelemetry();
  void start();
  void sendPadData(double x, double y, double z);

private:
  byte m_mac[6] = {
    0xA8, 0x61, 0x0A, 0xAE, 0x00, 0xB9
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