#include <Ethernet.h>
#include <EthernetUdp.h>

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xA8, 0x61, 0x0A, 0xAE, 0x00, 0xC3
};
IPAddress ip(192, 168, 0, 6);
IPAddress remoteIp(192, 168, 0, 5);

unsigned int udpPort = 8888;  // local port to listen on

// buffers for receiving and sending data
char ReplyBuffer[] = "acknowledged";  // a string to send back

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

uint16_t data1 = 5;
uint8_t data2 = 10;
double data3 = 10.5;

char buffer[11];

void createMessage() {
  data1++;
  data2++;
  data3++;
  memcpy(buffer, &data1, 2);
  memcpy(buffer + 2, &data2, 1);
  memcpy(buffer + 3, &data3, 8);
}

void setup() {

  // start the Ethernet
  Ethernet.begin(mac, ip);

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
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
  Udp.begin(udpPort);
}

void loop() {

  createMessage();
  // send a reply to the IP address and port that sent us the packet we received
  Udp.beginPacket(remoteIp, udpPort);
  Udp.write(buffer, 11);
  Udp.endPacket();
  // }
  delay(10);
}
