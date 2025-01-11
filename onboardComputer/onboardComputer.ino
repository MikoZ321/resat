#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <unordered_map>
#include <u-blox_config_keys.h>
#include <u-blox_structs.h>

using namespace std;

void getSensorData();
void printSensorData();

Adafruit_BME280 bme; // I2C
SFE_UBLOX_GNSS gnss; // I2C

// Pin definitions
const uint8_t PIN_LED_RED = 12;
const uint8_t PIN_LED_GREEN = 14;
const uint8_t PIN_LED_BLUE = 32;


const float SEA_LEVEL_PRESSURE_HPA = 1013.25;
const unsigned long DELAY_TIME = 1000;

unordered_map<char *, float> sensorData;

void setup() {
  // 115200 because of gps example
  Serial.begin(115200);

  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);

  // indicate that the CanSat is in low-power mode
  analogWrite(PIN_LED_RED, 0);
  analogWrite(PIN_LED_GREEN, 0);
  analogWrite(PIN_LED_BLUE, 255);

  bool status;

  // default settings
  status = bme.begin(0x77);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor.");
    // potentially dangerous if the BME280 gets damaged and the microcontroller restarts it won't be able function
    while (1);
  }
  status = gnss.begin(Serial);

  gnss.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  gnss.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  gnss.saveConfiguration(); //Save the current settings to flash and BBR
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

  // read M10Q data
  // possible to also read altitude to compare with the calculated one
  sensorData["latitude"] = gnss.getLatitude();
  sensorData["longitude"] = gnss.getLongitude();
}


// function only for testing will have no use in final code
void printSensorData() {
  for (auto& [key, value]: sensorData) {
    Serial.print(key);
    Serial.print(": ");
    Serial.println(value);
  }
}