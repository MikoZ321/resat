#include <Adafruit_ADS1X15.h>
#include <Adafruit_BME280.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <LSM9DS1TR-SOLDERED.h>
#include <SD.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <string>
#include <u-blox_config_keys.h>
#include <u-blox_structs.h>
#include <Wire.h>

using namespace std;

struct vector3D{
  float x;
  float y;
  float z;
};

struct dataContainer{
  long tickCount;
  float temperature;
  float pressure;
  float humidity;
  float altitude;
  long latitude;
  long longitude;
  int lightLevel;
  float batteryVoltage;
  float motorOutputVoltage;
  vector3D gyro;
  vector3D acceleration;
  vector3D magnetometer;
  float hallEffect;
};

string dataContainerToString(dataContainer input, string seperator);
void getSensorData();  
bool isDescending();
void printSensorData();
string vector3DToString(vector3D input, string seperator);

// Pin definitions
const uint8_t PIN_SD_CS = 14;
const uint8_t PIN_SERVO = 16;
const uint8_t PIN_LED_STRIP = 26;
const uint8_t PIN_HALL_SENSOR = 32;
const uint8_t ADC_MOTOR = 0;
const uint8_t ADC_BATTERY = 1;
const uint8_t ADC_PHOTORESISTOR = 2;

const float SEA_LEVEL_PRESSURE = 1013.25; // [pressure] = hPa
const unsigned int LED_COUNT = 4;

// Customizable settings
const unsigned int DELAY_TIME = 1000; // [delay time] = ms
const int MIN_ALTITUDE_DIFFERENCE = 5; // [altitude difference] = m
const int MIN_LIGHT_LEVEL = 500;
const int SERVO_ROTATION_ANGLE = 90; // [angle] = degree

// Init objects to control peripherals
Adafruit_ADS1115 ads; // I2C
Adafruit_BME280 bme; // I2C
Adafruit_NeoPixel ledStrip(LED_COUNT, PIN_LED_STRIP, NEO_GRB + NEO_KHZ800);
LSM9DS1TR lsm; // I2C
SFE_UBLOX_GNSS gnss; // UART
Servo myServo;

dataContainer sensorData;
// 0 - low power mode, 1 - normal mode
// always starts in low power mode
int currentMode = 0;
float previousAltitude = 0;
File outputFile;

void setup() {
  sensorData.tickCount = 0;
  // 115200 because of gps example
  Serial.begin(115200);
  Wire.begin();

  // show that the CanSat is powered on
  ledStrip.begin();
  for (int pixel = 0; pixel < LED_COUNT; pixel++) {
    ledStrip.setPixelColor(pixel, ledStrip.Color(0, 0, 255));
  }
  ledStrip.show();

  myServo.attach(PIN_SERVO);
  
  SD.begin(PIN_SD_CS);
  if (!SD.exists("/output.csv")) {
    outputFile = SD.open("/output.csv", FILE_WRITE);

    // init header
    outputFile.print("tickCount;temperature;pressure;humidity;altitude;latitude;longitude;lightLevel;batteryVoltage;motorOutputVoltage;");
    outputFile.println("gyroX;gyroY;gyroZ;accelerationX;accelerationY;accelerationZ;magnetometerX;magnetometerY;magnetometerZ;hallEffect");

    outputFile.close();
  }
  
  // init I2C for all peripherals
  ads.begin();
  bme.begin();
  lsm.begin();

  // M10Q GPS is connected to UART 0
  gnss.begin(Serial);

  gnss.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  gnss.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  gnss.saveConfiguration(); //Save the current settings to flash and BBR
}


void loop() {
  sensorData.tickCount++;
  previousAltitude = sensorData.altitude;

  getSensorData();

  if (!currentMode && isDescending()) {
    currentMode = 1;

    // deploy parachute and blades
    myServo.write(SERVO_ROTATION_ANGLE);

    for (int pixel = 0; pixel < LED_COUNT; pixel++) {
      ledStrip.setPixelColor(pixel, ledStrip.Color(0, 255, 0));
    }
    ledStrip.show();
  }
  else if (currentMode && !isDescending()) {
    currentMode = 0;

    for (int pixel = 0; pixel < LED_COUNT; pixel++) {
      ledStrip.setPixelColor(pixel, ledStrip.Color(0, 0, 255));
    }
    ledStrip.show();
  }

  // write data to SD 
  outputFile = SD.open("/output.csv", FILE_APPEND);
  outputFile.println(dataContainerToString(sensorData, ";").c_str());
  outputFile.close();

  printSensorData();
  delay(DELAY_TIME);
}


// this is a mess will try to fix
string dataContainerToString(dataContainer input, string seperator) {
  return (to_string(sensorData.tickCount) + seperator + to_string(sensorData.temperature) + seperator + to_string(sensorData.pressure) + seperator + 
          to_string(sensorData.humidity) + seperator + to_string(sensorData.altitude) + seperator + to_string(sensorData.latitude) + seperator + 
          to_string(sensorData.longitude) + seperator + to_string(sensorData.lightLevel) + seperator + to_string(sensorData.batteryVoltage) + seperator + 
          to_string(sensorData.motorOutputVoltage) + seperator + vector3DToString(sensorData.gyro, seperator) + seperator + vector3DToString(sensorData.acceleration, seperator) + seperator +
          vector3DToString(sensorData.magnetometer, seperator) + seperator + to_string(sensorData.hallEffect));
}


void getSensorData() {
  sensorData.temperature = bme.readTemperature(); // [temperature] = degrees Celcius
  sensorData.pressure = bme.readPressure(); // [pressure] = hPa
  sensorData.altitude = bme.readAltitude(SEA_LEVEL_PRESSURE); // [altitude] = m
  sensorData.humidity = bme.readHumidity(); // [humidity] = %

  // possible to also read altitude to compare with the calculated one
  // [latitude] = [longitude] = degrees * 10^7
  sensorData.latitude = gnss.getLatitude();
  sensorData.longitude = gnss.getLongitude();
  
  // all data from ADS1115IDGSR not scaled
  // TODO: make these readings useful
  // TODO: change keys when updated to proper units
  sensorData.batteryVoltage = ads.readADC_SingleEnded(ADC_BATTERY);
  sensorData.lightLevel = ads.readADC_SingleEnded(ADC_PHOTORESISTOR);

  if (currentMode) {
    // [gyro] = DPS
    lsm.readGyro();
    sensorData.gyro.x = lsm.calcGyro(lsm.gx);
    sensorData.gyro.y = lsm.calcGyro(lsm.gy);
    sensorData.gyro.z = lsm.calcGyro(lsm.gz);

    // [acceleration] = g
    lsm.readAccel();
    sensorData.acceleration.x = lsm.calcAccel(lsm.ax);
    sensorData.acceleration.y = lsm.calcAccel(lsm.ay);
    sensorData.acceleration.z = lsm.calcAccel(lsm.az);

    // [magnetometerData] = Gauss
    lsm.readMag();
    sensorData.magnetometer.x = lsm.calcMag(lsm.mx);
    sensorData.magnetometer.y = lsm.calcMag(lsm.my);
    sensorData.magnetometer.z = lsm.calcMag(lsm.mz);

    // all data from ADS1115IDGSR not scaled
    // TODO: make these readings useful
    // TODO: change keys when updated to proper units
    sensorData.motorOutputVoltage = ads.readADC_SingleEnded(ADC_MOTOR);

    // TODO: make Hall effect sensor actually useful
    sensorData.hallEffect = analogRead(PIN_HALL_SENSOR);
  }
}


bool isDescending() {
  return (previousAltitude - sensorData.altitude > MIN_ALTITUDE_DIFFERENCE) && (sensorData.lightLevel > MIN_LIGHT_LEVEL);
}


// function only for testing will have no use in final code, temporary solution will be fixed soon
void printSensorData() {
  Serial.println(dataContainerToString(sensorData, ";").c_str());
}


string vector3DToString(vector3D input, string seperator) {
  return (to_string(input.x) + seperator + to_string(input.y) + seperator + to_string(input.z));
}