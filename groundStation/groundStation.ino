#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
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
const char *ssid = "SSID";
const char *password = "PASSWORD";

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
 
  // Setup LoRa module
  LoRa.setPins(PIN_LORA_CS, PIN_LORA_RESET, PIN_LORA_DIO0);
 
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }

  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWITH);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);

  // WebSocket setup
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  // Serve Web Page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", getHTML());
  });

  server.begin();
}

String getHTML() {
    return R"rawliteral(
        <!DOCTYPE html>
        <html>
        <head>
            <title>ESP32 LoRa Live Data</title>
            <script>
                var ws = new WebSocket("ws://" + window.location.host + "/ws");
                ws.onmessage = function(event) {
                    document.getElementById("loradata").innerText = event.data;
                };
            </script>
        </head>
        <body>
            <h2>Live LoRa Data:</h2>
            <h3 id="loradata">Waiting for data...</h3>
        </body>
        </html>
    )rawliteral";
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