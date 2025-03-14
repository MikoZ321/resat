#include <SPI.h>
#include <LoRa.h>
 
// Define the pins used by the LoRa module
const int PIN_LORA_CS = 3;     // LoRa radio chip select
const int PIN_LORA_RESET = 5;  // LoRa radio reset
const int PIN_LORA_DIO0 = 4;    // Must be a hardware interrupt pin

// Customizable settings
// Should be moved to a different document for consistency with onboardComputer
const double LORA_FREQUENCY = 433E6; // also change in onboardComputer.ino
const double LORA_SIGNAL_BANDWITH = 125E3; // also change in onboardComputer.ino
const int LORA_SPREADING_FACTOR = 10; // also change in onboardComputer.ino
 
void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;
 
  // Setup LoRa module
  LoRa.setPins(PIN_LORA_CS, PIN_LORA_RESET, PIN_LORA_DIO0);
 
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }

  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWITH);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
}
 
void loop() {
 
  // Try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Received a packet
    Serial.print("Received '");
 
    // Read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }
 
    // Print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}