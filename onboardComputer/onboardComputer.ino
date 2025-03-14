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
  float angularSpeed; // [angular speed] = RPM
};

string dataContainerToString(dataContainer input, string seperator);
float digitalToAnalog(int digitalInput, int r1, int r2);
float getAngularSpeed();
void getSensorData();  
bool isDescending();
void printSensorData();
int16_t readTemperature();
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
const uint8_t PIN_POWER_SWITCH = 15;
const uint8_t PIN_SD_CS = 14;
const uint8_t PIN_SERVO = 16;

const unsigned int LED_COUNT = 4;
const float ADC_VOLTAGE_RANGE = 4.096; // [voltage range] = V
const float ADC_DIGITAL_RANGE = 32767.0;
const float BATTERY_R1 = 46.0; // [resistance] = kiloOhms
const float BATTERY_R2 = 10.0; // [resistance] = kiloOhms
const float MOTOR_R1 = 75.0; // [resistance] = kiloOhms
const float MOTOR_R2 = 10.0; // [resistance] = kiloOhms

// Customizable settings
const string CSV_SEPERATOR = ";";
const unsigned int DELAY_TIME = 1000; // [delay time] = ms
const double LORA_FREQUENCY = 433E6; // also change in groundStation.ino
const double LORA_SIGNAL_BANDWITH = 125E3; // also change in groundStation.ino
const int LORA_SPREADING_FACTOR = 10; // also change in groundStation.ino
const int MIN_ALTITUDE_DIFFERENCE = 5; // [altitude difference] = m
const int MIN_HALL_EFFECT = 2070;
const int MIN_LIGHT_LEVEL = 500;
const char *OUTPUT_FILE_NAME = "/onboardData.csv";
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
unsigned long currentTime = 0;
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
  if (!SD.exists(OUTPUT_FILE_NAME)) {
    outputFile = SD.open(OUTPUT_FILE_NAME, FILE_WRITE);

    // init header
    outputFile.print("tickCount;temperature;pressure;humidity;altitude;latitude;longitude;lightLevel;batteryVoltage;motorOutputVoltage;");
    outputFile.println("gyroX;gyroY;gyroZ;accelerationX;accelerationY;accelerationZ;angularSpeed");

    outputFile.close();
  }
  
  // communication setup
  LoRa.setPins(PIN_LORA_CS, PIN_LORA_RESET, PIN_LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)){
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWITH);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  
  // init I2C for all peripherals
  ads.begin();
  bme.begin();
  gpsConnection.begin();
  lsm.begin();

  ads.setGain(GAIN_TWOTHIRDS);

  pinMode(PIN_HALL_SENSOR, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  // draw power from battery
  pinMode(PIN_POWER_SWITCH, OUTPUT);
  digitalWrite(PIN_POWER_SWITCH, LOW);
}


void loop() {
  currentTime = millis();
  sensorData.tickCount++;
  previousAltitude = sensorData.altitude;

  getSensorData();

  // TODO: test when new hall sensor comes
  // sensorData.angularSpeed = getAngularSpeed();

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
  outputFile = SD.open(OUTPUT_FILE_NAME, FILE_APPEND);
  outputFile.println(dataContainerToString(sensorData, CSV_SEPERATOR).c_str());
  outputFile.close();
  
  // send data
  LoRa.beginPacket();
  LoRa.print(dataContainerToString(sensorData, CSV_SEPERATOR).c_str());
  LoRa.endPacket();

  printSensorData();
  
  while(millis() % DELAY_TIME) {
    ;
  }
}


// this is a mess will try to fix
string dataContainerToString(dataContainer input, string seperator) {
  return (to_string(sensorData.tickCount) + seperator + to_string(sensorData.temperature) + seperator + to_string(sensorData.pressure) + seperator + 
          to_string(sensorData.humidity) + seperator + to_string(sensorData.altitude) + seperator + to_string(sensorData.latitude) + seperator + 
          to_string(sensorData.longitude) + seperator + to_string(sensorData.lightLevel) + seperator + to_string(sensorData.batteryVoltage) + seperator + 
          to_string(sensorData.motorOutputVoltage) + seperator + vector3DToString(sensorData.gyro, seperator) + seperator + vector3DToString(sensorData.acceleration, seperator) +
          seperator + to_string(sensorData.angularSpeed));
}


float digitalToAnalog(int digitalInput, float r1, float r2) {
  float dividerRatio = r2 / (r1 + r2);
  return ((digitalInput * ADC_VOLTAGE_RANGE) / ADC_DIGITAL_RANGE) / dividerRatio;
}


float getAngularSpeed() {
  float lastRevolutionTime = currentTime;
  int revolutions = 0;
  bool peak = false;

  while (millis() - currentTime < 500) {
    float sensorData = analogRead(PIN_HALL_SENSOR);
    if (sensorData > MIN_HALL_EFFECT && !peak) {
      peak = true;
      revolutions++;
      lastRevolutionTime = millis();
    }

    if (sensorData < MIN_HALL_EFFECT) {
      peak = false;
    }
    delay(1);
  }

  Serial.print(currentTime);
  Serial.print(" ");
  Serial.print(lastRevolutionTime);
  Serial.print(" ");
  Serial.print(revolutions);
  Serial.print(" ");
  Serial.println((lastRevolutionTime - currentTime) / revolutions);

  return (revolutions > 0) * (60000.0/((lastRevolutionTime - currentTime) / revolutions));
}


void getSensorData() {
  bme.read(sensorData.pressure, sensorData.temperature, sensorData.humidity);

  int16_t tempRaw = readTemperature();
  
  // Convert raw temperature to degrees Celsius
  sensorData.temperature = tempRaw / 16.0 + 25.0;
  
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

  int digitalValue = ads.readADC_SingleEnded(ADC_BATTERY);
  sensorData.batteryVoltage = digitalToAnalog(digitalValue, BATTERY_R1, BATTERY_R2);

  sensorData.lightLevel = ads.readADC_SingleEnded(ADC_PHOTORESISTOR);

  lsm.readGyro();
  sensorData.gyro.x = lsm.calcGyro(lsm.gx);
  sensorData.gyro.y = lsm.calcGyro(lsm.gy);
  sensorData.gyro.z = lsm.calcGyro(lsm.gz);

  lsm.readAccel();
  sensorData.acceleration.x = lsm.calcAccel(lsm.ax);
  sensorData.acceleration.y = lsm.calcAccel(lsm.ay);
  sensorData.acceleration.z = lsm.calcAccel(lsm.az);

  digitalValue = ads.readADC_SingleEnded(ADC_MOTOR);
  sensorData.motorOutputVoltage = digitalToAnalog(digitalValue, MOTOR_R1, MOTOR_R2);
}


bool isDescending() {
  return (previousAltitude - sensorData.altitude > MIN_ALTITUDE_DIFFERENCE) && (sensorData.lightLevel > MIN_LIGHT_LEVEL);
}


// function only for testing will have no use in final code, temporary solution will be fixed soon
void printSensorData() {
  Serial.println(dataContainerToString(sensorData, ";").c_str());
}


int16_t readTemperature() {
  const int LSM9DS1_AG_ADDR  = 0x6B; // Accelerometer and gyroscope I2C address
  const int LSM9DS1_TEMP_OUT_L = 0x15; // Temperature output register (low byte)
  const int LSM9DS1_TEMP_OUT_H = 0x16; // Temperature output register (high byte)

  int16_t temp;
  
  // Read low and high bytes of temperature data
  Wire.beginTransmission(LSM9DS1_AG_ADDR);
  Wire.write(LSM9DS1_TEMP_OUT_L);
  Wire.endTransmission(false); // Restart
  Wire.requestFrom(LSM9DS1_AG_ADDR, 2);

  if (Wire.available() >= 2) {
    uint8_t tempL = Wire.read();
    uint8_t tempH = Wire.read();
    temp = (int16_t)((tempH << 8) | tempL);
  } else {
    Serial.println("Failed to read temperature data");
    temp = 0;
  }

  return temp;
}


string vector3DToString(vector3D input, string seperator) {
  return (to_string(input.x) + seperator + to_string(input.y) + seperator + to_string(input.z));
}