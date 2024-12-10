#include <SPI.h>
#include <LoRa.h>

//LoRa Pins
const uint8_t csPin = 11;
const uint8_t resetPin = 17;
const uint8_t dio0Pin = 2;

void setup() {
  Serial.begin(9600);
  while(!Serial)
    ;
  
  LoRa.setPins(csPin, resetPin, dio0Pin);

  if(!LoRa.begin(433E6)){
    Serial.println("Starting LoRa failed");
    while (1)
      ;
  }

}

void loop() {
  int uint8_t data = 0;

  //Send packet
  LoRa.beginPacket();
  LoRa.print(data);
  LoRa.endPacket();

  delay(1000);
}
