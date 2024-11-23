#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <unordered_map>

using namespace std;

void getSensorData();
void printSensorData();

Adafruit_BME280 bme; // I2C
const uint8_t PIN_LED_RED = 14;
const uint8_t PIN_LED_GREEN = 13;
const uint8_t PIN_LED_BLUE = 8;

const float SEA_LEVEL_PRESSURE_HPA = 1013.25;
const unsigned long DELAY_TIME = 1000;

unordered_map<char *, float> sensorData;

void setup() {
  Serial.begin(9600);

  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);

  // indicate that the CanSat is in low-power mode
  analogWrite(PIN_LED_RED, 0);
  analogWrite(PIN_LED_GREEN, 0);
  analogWrite(PIN_LED_BLUE, 255);

  bool status;

  // default settings
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor.");
    while (1);
  }
}


void loop() {
  getSensorData();
  printSensorData();
  delay(DELAY_TIME);
}


void getSensorData() {
  // read BME280 data
  sensorData["temperature"] = bme.readTemperature();
  sensorData["pressure"] = bme.readPressure();
  sensorData["altitude"] = bme.readAltitude(SEA_LEVEL_PRESSURE_HPA);
  sensorData["humidity"] = bme.readHumidity();
}


// function only for testing will have no use in final code
void printSensorData() {
  for (auto& [key, value]: sensorData) {
    Serial.print(key);
    Serial.print(": ");
    Serial.println(value);
  }
}