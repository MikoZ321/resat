#include <Adafruit_ADS1X15.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <BME280I2C.h>
#include <EnvironmentCalculations.h>
#include <ESP32Servo.h>
#include <LoRa.h>
#include <LSM9DS1TR-SOLDERED.h>
#include <SD.h>
#include <SparkFun_I2C_GPS_Arduino_Library.h>
#include <SPI.h>
#include <string>
#include <TinyGPS++.h>
#include <Wire.h>

using namespace std;

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
  double altitude; // [altitude] = m
  double latitude; // [latitude] = degrees N
  double longitude; // [longitude] = degrees E
  int lightLevel;
  float batteryVoltage; // [voltage] = V
  float motorOutputVoltage; // [voltage] = V
  vector3D gyro; // [gyro] = DPS
  vector3D acceleration; // [acceleration] = g
  vector3D magnetometer; // [magnetometer] = Gauss
  float hallEffect;
};

string dataContainerToString(dataContainer input, string seperator);
void getSensorData();  
bool isDescending();
void printSensorData();
string vector3DToString(vector3D input, string seperator);

// Pin definitions
const uint8_t ADC_BATTERY = 1;
const uint8_t ADC_MOTOR = 0;
const uint8_t ADC_PHOTORESISTOR = 2;
const uint8_t PIN_BUZZER = 25;
const uint8_t PIN_HALL_SENSOR = 32;
const uint8_t PIN_LED_STRIP = 26;
const uint8_t PIN_LORA_CS = 5;
const uint8_t PIN_LORA_DIO0 = 2;
const uint8_t PIN_LORA_RESET = 17;
const uint8_t PIN_SD_CS = 14;
const uint8_t PIN_SERVO = 16;

const unsigned int LED_COUNT = 4;
const float ADC_VOLTAGE_RANGE = 4.096; // [voltage range] = V
const float ADC_DIGITAL_RANGE = 32767.0;
const float BATTERY_DIVIDER_RATIO = 10.0 / 56.0;
const float MOTOR_DIVIDER_RATIO = 10.0 / 85.0;

// Customizable settings
const unsigned int DELAY_TIME = 1000; // [delay time] = ms
const int MIN_ALTITUDE_DIFFERENCE = 5; // [altitude difference] = m
const int MIN_LIGHT_LEVEL = 500;
const int SERVO_ROTATION_ANGLE = 90; // [angle] = degree

// Init objects to control peripherals
Adafruit_ADS1115 ads; // I2C
Adafruit_NeoPixel ledStrip(LED_COUNT, PIN_LED_STRIP, NEO_GRB + NEO_KHZ800);
BME280I2C bme; // I2C
LSM9DS1TR lsm; // I2C
I2CGPS gpsConnection; // I2C
TinyGPSPlus gps;
Servo myServo; // PWM

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
  
  // communication setup
  LoRa.setPins(PIN_LORA_CS, PIN_LORA_RESET, PIN_LORA_DIO0);
  if (!LoRa.begin(433E6)){
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  
  // init I2C for all peripherals
  ads.begin();
  bme.begin();
  gpsConnection.begin();
  lsm.begin();

  ads.setGain(GAIN_TWOTHIRDS);

  pinMode(PIN_HALL_SENSOR, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
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

    digitalWrite(PIN_BUZZER, HIGH);

    for (int pixel = 0; pixel < LED_COUNT; pixel++) {
      ledStrip.setPixelColor(pixel, ledStrip.Color(0, 0, 255));
    }
    ledStrip.show();
  }

  // write data to SD 
  outputFile = SD.open("/output.csv", FILE_APPEND);
  outputFile.println(dataContainerToString(sensorData, ";").c_str());
  outputFile.close();
  
  // send data
  LoRa.beginPacket();
  LoRa.print(dataContainerToString(sensorData, ";").c_str());
  LoRa.endPacket();

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
  bme.read(sensorData.pressure, sensorData.temperature, sensorData.humidity);
  
  while (gpsConnection.available()) //available() returns the number of new bytes available from the GPS module
  {
    gps.encode(gpsConnection.read()); //Feed the GPS parser
  }

  if (gps.location.isValid()) {
    sensorData.latitude = gps.location.lat();
    sensorData.longitude = gps.location.lng();
  }

  if (gps.altitude.isValid()) {
    sensorData.altitude = gps.altitude.meters();
  }
  else {
    sensorData.altitude = EnvironmentCalculations::Altitude(sensorData.pressure);
  }
  
  sensorData.batteryVoltage = ((ads.readADC_SingleEnded(ADC_BATTERY) * ADC_VOLTAGE_RANGE) / ADC_DIGITAL_RANGE) / BATTERY_DIVIDER_RATIO;
  sensorData.lightLevel = ads.readADC_SingleEnded(ADC_PHOTORESISTOR);

  lsm.readGyro();
  sensorData.gyro.x = lsm.calcGyro(lsm.gx);
  sensorData.gyro.y = lsm.calcGyro(lsm.gy);
  sensorData.gyro.z = lsm.calcGyro(lsm.gz);

  lsm.readAccel();
  sensorData.acceleration.x = lsm.calcAccel(lsm.ax);
  sensorData.acceleration.y = lsm.calcAccel(lsm.ay);
  sensorData.acceleration.z = lsm.calcAccel(lsm.az);

  lsm.readMag();
  sensorData.magnetometer.x = lsm.calcMag(lsm.mx);
  sensorData.magnetometer.y = lsm.calcMag(lsm.my);
  sensorData.magnetometer.z = lsm.calcMag(lsm.mz);

  sensorData.motorOutputVoltage = ((ads.readADC_SingleEnded(ADC_MOTOR) * ADC_VOLTAGE_RANGE) / ADC_DIGITAL_RANGE) / MOTOR_DIVIDER_RATIO;

  // TODO: make Hall effect sensor actually useful
  sensorData.hallEffect = analogRead(PIN_HALL_SENSOR);
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