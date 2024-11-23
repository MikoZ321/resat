#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <string>
#include <unordered_map>

using namespace std;

unordered_map<char *,float> getSensorData();
void printSensorData(unordered_map<char *,float> sensorData);

Adafruit_BME280 bme; // I2C

const float SEA_LEVEL_PRESSURE_HPA = 1013.25;
const unsigned long DELAY_TIME = 1000;

void setup() {
  Serial.begin(9600);

  bool status;

  // default settings
  status = bme.begin();  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor.");
    while (1);
  }
}


void loop() {
  unordered_map<char *, float> sensorData;
  sensorData = getSensorData();

  printSensorData(sensorData);
  delay(DELAY_TIME);
}


unordered_map<char *,float> getSensorData() {
  unordered_map<char *,float> sensorData;

  // read BME280 data
  sensorData["temperature"] = bme.readTemperature();
  sensorData["pressure"] = bme.readPressure();
  sensorData["altitude"] = bme.readAltitude(SEA_LEVEL_PRESSURE_HPA);
  sensorData["humidity"] = bme.readHumidity();

  return sensorData;
}


// function only for testing will have no use in final code
void printSensorData(unordered_map<char *,float> sensorData) {
  for (auto& [key, value]: sensorData) {
    Serial.print(key);
    Serial.print(": ");
    Serial.println(value);
  }
}