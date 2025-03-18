#include <Adafruit_ADS1X15.h>
#include <Adafruit_BME280.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <array>
#include <charconv>
#include <LoRa.h>
#include <LSM9DS1TR-SOLDERED.h>
#include <SD.h>
#include <SparkFun_I2C_GPS_Arduino_Library.h>
#include <SparkFun_TMP117.h>
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

string dataContainerToString(dataContainer input, string seperator);
float digitalToAnalog(int digitalInput, int r1, int r2);
float getAngularSpeed();
void getSensorData();  
bool isDescending();
string toStringWithPrecision(double value);
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
const int LORA_SPREADING_FACTOR = 8; // also change in groundStation.ino
const int MIN_ALTITUDE_DIFFERENCE = 5; // [altitude difference] = m
const int MIN_HALL_EFFECT = 2070;
const int MIN_LIGHT_LEVEL = 500;
const char *OUTPUT_FILE_NAME = "/onboardData.csv";
const int VALUE_ACCURACY = 4;

// Init objects to control peripherals
Adafruit_ADS1115 ads; // I2C
Adafruit_NeoPixel ledStrip(LED_COUNT, PIN_LED_STRIP, NEO_GRB + NEO_KHZ800);
Adafruit_BME280 bme; // I2C
LSM9DS1TR lsm; // I2C
I2CGPS gpsConnection; // I2C
TinyGPSPlus gps;
TMP117 tmp117;

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
  
  SD.begin(PIN_SD_CS);
  if (!SD.exists(OUTPUT_FILE_NAME)) {
    outputFile = SD.open(OUTPUT_FILE_NAME, FILE_WRITE);

    // init header
    outputFile.print("tickCount;temperature;pressure;humidity;altitudeGPS;altitudePressure;latitude;longitude;lightLevel;batteryVoltage;motorOutputVoltage;");
    outputFile.println("gyroX;gyroY;gyroZ;accelerationX;accelerationY;accelerationZ;angularSpeed");

    outputFile.close();
  }
  
  // communication setup
  LoRa.setPins(PIN_LORA_CS, PIN_LORA_RESET, PIN_LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)){
    Serial.println("Starting LoRa failed!");
  }

  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWITH);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  
  // init I2C for all peripherals
  ads.begin();
  bme.begin(0x76);
  gpsConnection.begin();
  lsm.begin();
  tmp117.begin(0x49);

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
  previousAltitude = sensorData.altitudePressure;

  getSensorData();

  // TODO: make faster, takes 2-3 seconds has to take 1
  sensorData.angularSpeed = getAngularSpeed();

  if (!currentMode && isDescending()) {
    currentMode = 1;

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
  
  while(millis() % DELAY_TIME) {
    ;
  }
}


// this is a mess will try to fix
string dataContainerToString(dataContainer input, string separator) {
  return (to_string(sensorData.tickCount) + separator +
          toStringWithPrecision(sensorData.temperature) + separator +
          toStringWithPrecision(sensorData.pressure) + separator +
          toStringWithPrecision(sensorData.humidity) + separator +
          toStringWithPrecision(sensorData.altitudeGPS) + separator +
          toStringWithPrecision(sensorData.altitudePressure) + separator +
          to_string(sensorData.latitude) + separator +  
          to_string(sensorData.longitude) + separator + 
          toStringWithPrecision(sensorData.lightLevel) + separator +
          toStringWithPrecision(sensorData.batteryVoltage) + separator +
          toStringWithPrecision(sensorData.motorOutputVoltage) + separator +
          vector3DToString(sensorData.gyro, separator) + separator +
          vector3DToString(sensorData.acceleration, separator) + separator +
          toStringWithPrecision(sensorData.angularSpeed));
}


float digitalToAnalog(int digitalInput, float r1, float r2) {
  float dividerRatio = r2 / (r1 + r2);
  return ((digitalInput * ADC_VOLTAGE_RANGE) / ADC_DIGITAL_RANGE) / dividerRatio;
}


float getAngularSpeed() {
  Serial.println(currentTime);
  float lastRevolutionTime = currentTime;
  float period = 0;
  int revolutions = 0;
  bool peak = false;

  while (millis() - currentTime < 500 && revolutions < 2) {
    float sensorData = analogRead(PIN_HALL_SENSOR);
    
    if (sensorData > MIN_HALL_EFFECT && !peak) {
      peak = true;
      revolutions++;
      period = millis() - lastRevolutionTime;
      lastRevolutionTime = millis();
    }

    if (sensorData < MIN_HALL_EFFECT) {
      peak = false;
    }
    delay(1);
  }

  if (revolutions == 0) {
    return 0;
  }
  return 60000.0/(period);
}


void getSensorData() {
  sensorData.temperature = tmp117.readTempC();

  sensorData.pressure = bme.readPressure() / 100.0F; // Convert to hPa
  sensorData.altitudePressure = bme.readAltitude(1013.25); // Absolute altitude

  while (gpsConnection.available()) //available() returns the number of new bytes available from the GPS module
  {
    gps.encode(gpsConnection.read()); //Feed the GPS parser
  }

  if (gps.location.isValid()) {
    sensorData.latitude = gps.location.lat();
    sensorData.longitude = gps.location.lng();
    sensorData.altitudeGPS = gps.altitude.meters();
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
  return (previousAltitude - sensorData.altitudeGPS > MIN_ALTITUDE_DIFFERENCE) && (sensorData.lightLevel > MIN_LIGHT_LEVEL);
}

string toStringWithPrecision(double value) {
  array<char, 16> buffer{};  // Buffer large enough for most double values
  auto [ptr, ec] = to_chars(buffer.data(), buffer.data() + buffer.size(), value, chars_format::fixed, VALUE_ACCURACY);
  return (ec == errc()) ? string(buffer.data(), ptr) : "NaN";
}


string vector3DToString(vector3D input, string seperator) {
  return (toStringWithPrecision(input.x) + seperator + toStringWithPrecision(input.y) + seperator + toStringWithPrecision(input.z));
}