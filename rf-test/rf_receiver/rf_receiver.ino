/*
* Arduino Wireless Communication Tutorial
*       Example 1 - Receiver Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(8, 9); // CE, CSN

const byte address[6] = "00001";

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    float data[] = {0,0};

    // uint8_t bytes[8];
    float bytes[2];
    radio.read(&bytes, sizeof(float) * 2);
    // Serial.write("start");
    Serial.write(bytes[0]);
    Serial.write(" ");
    Serial.write(bytes[1]);
    // Serial.write("end");
    // Serial.readBytes(&data, sizeof(data));
    // Serial.println(text);
    // Serial.write(data);
  }

}
