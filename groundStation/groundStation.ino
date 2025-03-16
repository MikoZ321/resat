#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <LoRa.h>
#include <SPI.h>
#include <WiFi.h>

struct vector3D {
  float x;
  float y;
  float z;
};

struct dataContainer {
  long tickCount;
  float temperature; // [temperature] = degrees Celcius
  float pressure; // [pressure] = hPa
  float humidity; // [humidity] = % RH
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
};

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

dataContainer receivedData;

String dataToJSON(void);
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void sendWebSocketMessage(String message);
void parseLoRaData(String rawData);
 
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
 
    parseLoRaData(rawData);
    String jsonData = dataToJSON();
    sendWebSocketMessage(jsonData);
  }
  ws.cleanupClients();
}


String dataToJSON(void) {
  String jsonData = "{";
  jsonData += "\"tickCount\":" + String(receivedData.tickCount) + ",";
  jsonData += "\"temperature\":" + String(receivedData.temperature) + ",";
  jsonData += "\"pressure\":" + String(receivedData.pressure) + ",";
  jsonData += "\"humidity\":" + String(receivedData.humidity) + ",";
  jsonData += "\"altitudeGPS\":" + String(receivedData.altitudeGPS) + ",";
  jsonData += "\"altitudePressure\":" + String(receivedData.altitudePressure) + ",";
  jsonData += "\"latitude\":" + String(receivedData.latitude) + ",";
  jsonData += "\"longitude\":" + String(receivedData.longitude) + ",";
  jsonData += "\"lightLevel\":" + String(receivedData.lightLevel) + ",";
  jsonData += "\"batteryVoltage\":" + String(receivedData.batteryVoltage) + ",";
  jsonData += "\"motorOutputVoltage\":" + String(receivedData.motorOutputVoltage) + ",";
  jsonData += "\"gyro\":{\"x\":" + String(receivedData.gyro.x) + ",\"y\":" + String(receivedData.gyro.y) + ",\"z\":" + String(receivedData.gyro.z) + "},";
  jsonData += "\"acceleration\":{\"x\":" + String(receivedData.acceleration.x) + ",\"y\":" + String(receivedData.acceleration.y) + ",\"z\":" + String(receivedData.acceleration.z) + "},";
  jsonData += "\"angularSpeed\":" + String(receivedData.angularSpeed);
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
  char buf[rawData.length() + 1];
  rawData.toCharArray(buf, sizeof(buf));

  char *token = strtok(buf, ";");
  if (token == NULL) return;

  receivedData.tickCount = atol(token);

  token = strtok(NULL, ";");
  receivedData.temperature = atof(token);

  token = strtok(NULL, ";");
  receivedData.pressure = atof(token);

  token = strtok(NULL, ";");
  receivedData.humidity = atof(token);

  token = strtok(NULL, ";");
  receivedData.altitudeGPS = atof(token);

  token = strtok(NULL, ";");
  receivedData.altitudePressure = atof(token);

  token = strtok(NULL, ";");
  receivedData.latitude = atof(token);

  token = strtok(NULL, ";");
  receivedData.longitude = atof(token);

  token = strtok(NULL, ";");
  receivedData.lightLevel = atoi(token);

  token = strtok(NULL, ";");
  receivedData.batteryVoltage = atof(token);

  token = strtok(NULL, ";");
  receivedData.motorOutputVoltage = atof(token);

  token = strtok(NULL, ";");
  receivedData.gyro.x = atof(token);
  
  token = strtok(NULL, ";");
  receivedData.gyro.y = atof(token);
  
  token = strtok(NULL, ";");
  receivedData.gyro.z = atof(token);
  
  token = strtok(NULL, ";");
  receivedData.acceleration.x = atof(token);
  
  token = strtok(NULL, ";");
  receivedData.acceleration.y = atof(token);
  
  token = strtok(NULL, ";");
  receivedData.acceleration.z = atof(token);
  
  token = strtok(NULL, ";");
  receivedData.angularSpeed = atof(token);
}