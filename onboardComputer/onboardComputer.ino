#include <Adafruit_ADS1X15.h>
#include <Adafruit_BME280.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <LSM9DS1TR-SOLDERED.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <unordered_map>
#include <u-blox_config_keys.h>
#include <u-blox_structs.h>
#include <Wire.h>

using namespace std;

void getSensorData();
void printSensorData();

// Pin definitions
const uint8_t PIN_LED_STRIP = 26;
const uint8_t ADC_MOTOR = 0;
const uint8_t ADC_BATTERY = 1;
const uint8_t ADC_PHOTORESISTOR = 2;

const float SEA_LEVEL_PRESSURE_HPA = 1013.25;
const unsigned int LED_COUNT = 4;

// Customizable settings
const unsigned long DELAY_TIME = 1000;

// Init objects to control peripherals
Adafruit_ADS1115 ads; // I2C
Adafruit_BME280 bme; // I2C
Adafruit_NeoPixel ledStrip(LED_COUNT, PIN_LED_STRIP, NEO_GRB + NEO_KHZ800);
LSM9DS1TR lsm; // I2C
SFE_UBLOX_GNSS gnss; // UART

unordered_map<char *, float> sensorData;

void setup() {
  // 115200 because of gps example
  Serial.begin(115200);

  Wire.begin();
  bool status;

  // show that the CanSat is powered on
  ledStrip.begin();
  for (int pixel = 0; pixel < LED_COUNT; pixel++) {
    ledStrip.setPixelColor(pixel, ledStrip.Color(255, 0, 0));
  }
  ledStrip.show();

  ads.begin();

  // default settings
  status = bme.begin(0x77);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor.");
    // potentially dangerous if the BME280 gets damaged and the microcontroller restarts it won't be able function
    while (1);
  }

  status = lsm.begin();
  if (!status) {
    Serial.println("Could not connect to LSM9DS1TR.");
    while (1);
  }

  // M10Q GPS is connected to UART 0
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
  // [temperature] = degrees Celcius
  sensorData["temperature"] = bme.readTemperature();
  // [pressure] = hPa
  sensorData["pressure"] = bme.readPressure();
  // [altitude] = m
  sensorData["altitude"] = bme.readAltitude(SEA_LEVEL_PRESSURE_HPA);
  // [humidity] = %
  sensorData["humidity"] = bme.readHumidity();

  // update lsm object with LSM9DS1TR data
  // [gyro] = DPS
  lsm.readGyro();
  sensorData["gyroX"] = lsm.calcGyro(lsm.gx);
  sensorData["gyroY"] = lsm.calcGyro(lsm.gy);
  sensorData["gyroZ"] = lsm.calcGyro(lsm.gz);

  // [acceleration] = g
  lsm.readAccel();
  sensorData["accelerationX"] = lsm.calcAccel(lsm.ax);
  sensorData["accelerationY"] = lsm.calcAccel(lsm.ay);
  sensorData["accelerationZ"] = lsm.calcAccel(lsm.az);

  // [magnetometerData] = Gauss
  lsm.readMag();
  sensorData["magnetometerX"] = lsm.calcMag(lsm.mx);
  sensorData["magnetometerY"] = lsm.calcMag(lsm.my);
  sensorData["magnetometerZ"] = lsm.calcMag(lsm.mz);

  // all data from ADS1115IDGSR not scaled
  // TODO: change keys when updated to proper units
  sensorData["motor"] = ads.readADC_SingleEnded(ADC_MOTOR);
  sensorData["battery"] = ads.readADC_SingleEnded(ADC_BATTERY);
  sensorData["lightLevel"] = ads.readADC_SingleEnded(ADC_PHOTORESISTOR);

  // read M10Q data
  // possible to also read altitude to compare with the calculated one
  // [latitude] = [longitude] = degrees * 10^7
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