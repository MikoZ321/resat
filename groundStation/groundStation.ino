#include <AsyncTCP.h>
#include <cmath>
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
#define BASE_LATITUDE 0
#define BASE_LONGITUDE 0
#define EARTH_RADIUS 6371000
#define LORA_FREQUENCY 433E6 // also change in onboardComputer.ino
#define LORA_SIGNAL_BANDWITH 125E3 // also change in onboardComputer.ino
#define LORA_SPREADING_FACTOR 8 // also change in onboardComputer.ino
#define LORA_ITEM_COUNT 12
#define PI 3.141592653589793

// Wi-Fi credentials
#define SSID "Kontyner"
#define PASSWORD "Kancelaria2012"

/*struct vector3D {
  float x;
  float y;
  float z;
};

struct dataContainer {
  long time; // [time] = ms
  float temperature; // [temperature] = degrees Celcius
  float pressure; // [pressure] = hPa
  float altitudePressure; // [altitude] = m
  float latitude; // [latitude] = degrees N
  float longitude; // [longitude] = degrees E
  float batteryVoltage; // [voltage] = V
  float motorOutputVoltage; // [voltage] = V
  vector3D acceleration; // [acceleration] = g
  float angularSpeed; // [angular speed] = RPM
};*/

AsyncWebServer server(80);
AsyncWebSocket ws("/ws"); // WebSocket endpoint

// parsed data is stored in this str arr in the order of the dataContainer struct
String receivedData[LORA_ITEM_COUNT];
float currentHeight = 0;
float previousHeight = 0;
long previousTime = 0;

String dataToJSON(void);
double flatEarthDistance(double lat1, double lon1, float height);
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void sendWebSocketMessage(String message);
double toRadians(double angle);
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
    currentHeight = 44330 * (1 - pow((String(receivedData[2]).toFloat()/1013.25), (1/5.225))) - BASE_HEIGHT;

    String jsonData = dataToJSON();

    //Serial.println(jsonData);
    sendWebSocketMessage(jsonData);  
  }
  ws.cleanupClients();
  previousHeight = currentHeight;
}


String dataToJSON(void) {
  String jsonData = "{";
  jsonData += "\"time\":" + receivedData[0] + ",";
  jsonData += "\"temperature\":" + receivedData[1] + ",";
  jsonData += "\"pressure\":" + receivedData[2] + ",";
  jsonData += "\"altitudeGPS\":" + receivedData[3] + ",";
  jsonData += "\"height\":" + String(currentHeight) + ",";
  jsonData += "\"latitude\":" + receivedData[4] + ",";
  jsonData += "\"longitude\":" + receivedData[5] + ",";
  jsonData += "\"batteryVoltage\":" + receivedData[6] + ",";
  jsonData += "\"motorOutputVoltage\":" + receivedData[7] + ",";
  jsonData += "\"accelX\":" + receivedData[8] + ",";
  jsonData += "\"accelY\":" + receivedData[9] + ",";
  jsonData += "\"accelZ\":" + receivedData[10] + ",";
  jsonData += "\"angularSpeed\":" + receivedData[11] + ",";
  jsonData += "\"descentRate\":" + String(previousHeight - currentHeight) + ",";
  jsonData += "\"distance\":" + String(flatEarthDistance((String(receivedData[4])).toFloat(), (String(receivedData[5])).toFloat(), currentHeight)) + ",";
  jsonData += "\"rssi\":" + String(LoRa.packetRssi());
  jsonData += "}";
  return jsonData;
}


double flatEarthDistance(double lat1, double lon1, float height) {
  double avgLat = toRadians((lat1 + BASE_LATITUDE) / 2);
  double dx = (toRadians(BASE_LONGITUDE - lon1)) * cos(avgLat) * EARTH_RADIUS;
  double dy = (toRadians(BASE_LATITUDE - lat1)) * EARTH_RADIUS;
  return sqrt(dx * dx + dy * dy + height * height); // [distance] = m
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


double toRadians(double angle) {
  return angle * PI / 180.0;
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
        receivedData[currentIndex] = String(buffer);
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