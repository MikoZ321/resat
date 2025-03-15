#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <LoRa.h>
#include <SPI.h>
#include <WiFi.h>

// Pin definitions
const int PIN_LORA_CS = 3;     // LoRa radio chip select
const int PIN_LORA_RESET = 5;  // LoRa radio reset
const int PIN_LORA_DIO0 = 4;    // Must be a hardware interrupt pin

// Customizable settings
// Should be moved to a different document for consistency with onboardComputer
const double LORA_FREQUENCY = 433E6; // also change in onboardComputer.ino
const double LORA_SIGNAL_BANDWITH = 125E3; // also change in onboardComputer.ino
const int LORA_SPREADING_FACTOR = 10; // also change in onboardComputer.ino

// Wi-Fi credentials
const char *ssid = "Sosnowe Wzgorze";
const char *password = "9349055587";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws"); // WebSocket endpoint

String receivedData = "No Data Yet"; // Store received LoRa data

// WebSocket event handler
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("WebSocket client connected");
    client->text(receivedData); // Send current data to new client
  }
}

// Send LoRa data to all WebSocket clients
void sendWebSocketMessage(String message) {
  ws.textAll(message);
}
 
void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;

  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi, IP: " + WiFi.localIP().toString());

  if (!LittleFS.begin(true)) {
    Serial.println("An error occurred while mounting LittleFS.");
    return;
  }
 
  // Setup LoRa module
  /**LoRa.setPins(PIN_LORA_CS, PIN_LORA_RESET, PIN_LORA_DIO0);
 
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }

  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWITH);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);**/

  // WebSocket setup
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  // Serve HTML, CSS, and JS from SPIFFS
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/style.css", "text/css");
  });
  server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/script.js", "application/javascript");
  });

  server.begin();
}
 
void loop() {
  delay(1000);
  // Try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Received a packet
    Serial.print("Received '");
 
    // Read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
      receivedData += (char)LoRa.read();
    }

    Serial.println("Received LoRa Data: " + receivedData);
    sendWebSocketMessage(receivedData);
 
    // Print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
  ws.cleanupClients();
}