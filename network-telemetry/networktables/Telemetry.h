#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Ethernet.h>
#include <EthernetUdp.h>

namespace
{
  // Enter a MAC address and IP address for your controller below.
  // The IP address will be dependent on your local network:
  byte mac[] = {
      0xA8, 0x61, 0x0A, 0xAE, 0x00, 0xC3};
  IPAddress ip(192, 168, 0, 6);
  IPAddress remoteIp(192, 168, 0, 5);

  unsigned int udpPort = 8888; // local port to listen on
}

class Telemetry
{
public:
  Telemetry() {}

  void start()
  {
    // start the Ethernet
    Ethernet.begin(mac, ip);
  }

  bool hasEthernetShield()
  {
    return Ethernet.hardwareStatus() != EthernetNoHardware;
  }

  bool isEthernetCableConnected()
  {
    return Ethernet.linkStatus() != LinkOFF;
  }

  bool isReady()
  {
    return hasEthernetShield() && isEthernetCableConnected();
  }

  void send(uint8_t messageId, uint8_t *message, uint8_t messageLength)
  {
    // send a reply to the IP address and port that sent us the packet we received
    udp_.beginPacket(remoteIp, udpPort);
    udp_.write(&messageId, 1);
    udp_.write(message, messageLength);
    udp_.endPacket();
  }

private:
  EthernetUDP udp_;
};

#endif // TELEMETRY_H