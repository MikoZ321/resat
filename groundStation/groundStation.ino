#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <LoRa.h>
#include <SPI.h>
#include <WiFi.h>

// Pin definitions
#define PIN_LORA_CS 5     // LoRa radio chip select
#define PIN_LORA_RESET 17 // LoRa radio reset
#define PIN_LORA_DIO0 14   // Must be a hardware interrupt pin

// Customizable settings
// Should be moved to a different document for consistency with onboardComputer
#define BASE_HEIGHT 135
#define LORA_FREQUENCY 433E6 // also change in onboardComputer.ino
#define LORA_SIGNAL_BANDWITH 125E3 // also change in onboardComputer.ino
#define LORA_SPREADING_FACTOR 8 // also change in onboardComputer.ino
#define LORA_ITEM_COUNT 18

// Wi-Fi credentials
#define SSID "Kontyner"
#define PASSWORD "Kancelaria2012"

/*struct vector3D {
  float x;
  float y;
  float z;
};

struct dataContainer {
  long tickCount;
  long time; // [time] = ms
  float temperature; // [temperature] = degrees Celcius
  float pressure; // [pressure] = hPa
  float altitudeGPS; // [altitude] = m
  float altitudePressure; // [altitude] = m
  float latitude; // [latitude] = degrees N
  float longitude; // [longitude] = degrees E
  int lightLevel;
  float batteryVoltage; // [voltage] = V
  float motorOutputVoltage; // [voltage] = V
  vector3D gyro; // [gyro] = DPS
  vector3D acceleration; // [acceleration] = g
  float angularSpeed; // [angular speed] = RPM
};*/

AsyncWebServer server(80);
AsyncWebSocket ws("/ws"); // WebSocket endpoint

// parsed data is stored in this str arr in the order of the dataContainer struct
String receivedData[LORA_ITEM_COUNT];
float previousHeight = 0;
long previousTime = 0;

String dataToJSON(void);
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void sendWebSocketMessage(String message);
void parseLoRaData(String rawData);
 
void setup() {
  Serial.begin(9600);

  WiFi.begin(SSID, PASSWORD);
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

  // Serve HTML, CSS, and JS from Little FS
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/style.css", "text/css");
  });
  server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/script.js", "application/javascript");
  });
  server.on("/cyberfreight.otf", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/cyberfreight.otf", "font/otf");
  });

  server.begin();
}

 
void loop() {
  // Try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String rawData = "";
    while (LoRa.available()) {
      rawData += (char)LoRa.read();
    }
    Serial.print("RSSI: ");
    Serial.print(LoRa.packetRssi());

    Serial.print(" ");
    Serial.println(rawData);
 
    parseLoRaData(rawData);
    String jsonData = dataToJSON();

    //Serial.println(jsonData);
    sendWebSocketMessage(jsonData);  
  }
  ws.cleanupClients();
  previousHeight = (String(receivedData[5])).toFloat();
}


String dataToJSON(void) {
  String jsonData = "{";
  jsonData += "\"tickCount\":" + receivedData[0] + ",";
  jsonData += "\"time\":" + receivedData[1] + ",";
  jsonData += "\"temperature\":" + receivedData[2] + ",";
  jsonData += "\"pressure\":" + receivedData[3] + ",";
  jsonData += "\"altitudeGPS\":" + receivedData[4] + ",";
  jsonData += "\"height\":" + String(receivedData[5]) + ",";
  jsonData += "\"latitude\":" + receivedData[6] + ",";
  jsonData += "\"longitude\":" + receivedData[7] + ",";
  jsonData += "\"lightLevel\":" + receivedData[8] + ",";
  jsonData += "\"batteryVoltage\":" + receivedData[9] + ",";
  jsonData += "\"motorOutputVoltage\":" + receivedData[10] + ",";
  jsonData += "\"gyroX\":" + receivedData[11] + ",";
  jsonData += "\"gyroY\":" + receivedData[12] + ",";
  jsonData += "\"gyroZ\":" + receivedData[13] + ",";
  jsonData += "\"accelX\":" + receivedData[14] + ",";
  jsonData += "\"accelY\":" + receivedData[15] + ",";
  jsonData += "\"accelZ\":" + receivedData[16] + ",";
  jsonData += "\"angularSpeed\":" + receivedData[17] + ",";
  jsonData += "\"descentRate\":" + String(previousHeight - (String(receivedData[5])).toFloat()) + ",";
  jsonData += "\"rssi\":" + String(LoRa.packetRssi());
  jsonData += "}";
  return jsonData;
}


// WebSocket event handler
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.println("WebSocket client connected");
      client->text("Connected to ESP32 WebSocket server!");
      break;

    case WS_EVT_DISCONNECT:
      Serial.println("WebSocket client disconnected");
      break;

    case WS_EVT_DATA:
      Serial.println("Received WebSocket data:");
      Serial.write(data, len);
      Serial.println();
      break;
          
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}


// Send LoRa data to all WebSocket clients
void sendWebSocketMessage(String message) {
  ws.textAll(message);
}


void parseLoRaData(String rawData) {
  char stringArray[rawData.length() + 1];
  rawData.toCharArray(stringArray, sizeof(stringArray));

  int currentIndex = 0;
  char buffer[32];  // Use a char array for better memory management
  int bufIndex = 0;

  for (int i = 0; i <= rawData.length(); i++) {  // Use regular loop
    char character = stringArray[i];

    if (character == ';' || character == '\0') {
      buffer[bufIndex] = '\0';  // Null-terminate the buffer
      if (currentIndex < LORA_ITEM_COUNT) {  // Prevent buffer overflow
        if (currentIndex == 5) {
          receivedData[currentIndex] = String((String(buffer)).toFloat() - BASE_HEIGHT);
        }
        else {
          receivedData[currentIndex] = String(buffer);
        }
        currentIndex++;
      }
      bufIndex = 0;  // Reset buffer index
    } else {
      if (bufIndex < sizeof(buffer) - 1) {  // Prevent buffer overflow
        buffer[bufIndex++] = character;
      }
    }
  }
}