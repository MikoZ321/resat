# ReSat
This repository contains all of the code used by the ReSAT Team taking part in the 2024/2025 European CanSat Competition.

## Hardware
This project will use two seperate microcontrollers: one onboard the CanSat and the other at the ground station.

### Onboard Computer
The onboard computer will be a costume-made PCB using the ESP-WROOM-32E it will be connected to the following peripherals:
* BME280 - temperature, atmospheric pressure, and air humidity sensor
* M10Q GPS QMC5883 - GPS module
* LSM9DS1 - accelerometer, and tilt sensor
* HX6659IUA-B - Hall effect sensor, used to determine the number of rotations of the parachute relative to the CanSat body
* Voltage divider - voltage divider, used to determine the amount of energy produced
* Ra-02 - LoRa communication module, used to transmit data to the ground station
* TF PUSH - TF card reader, used to write data to the micro-SD card onboard
* Servo micro 9g - microservo, used to free the parachute compartment
* Buzzer 2700Hz - buzzer, used to help locate the CanSat after landing
* FM-3510RGBA - RGB led, used to indicate the current mode of the CanSat

## Setup Process
This project was coded in Arduino IDE using the esp32 board library by Espressif (for a step-by-step setup process consult the link https://dronebotworkshop.com/esp32-intro/)

Libraries used:
* [BME280 by Adafruit](https://github.com/adafruit/Adafruit_BME280_Library/tree/master) 

## Software Architecture and Design
This project will consist of two main parts: 
1. The program which is to be executed by the onboard computer.
2. The ground station program.

### Onboard Software
Aims:
* Collect data from different sensors.
* Encode the data to an onboard micro-SD card.
* Transmit the data via Lo-Ra to the ground station.
* Ensure that the parachute compartment (along with the parachute and blades) is released via servo.

![Flowchart of the onboard program flow](./schematics/softwareExecutionPath.png)

A is the low-power mode and B is the normal mode.

![Schematic of the onboard data handling](./schematics/dataHandling.png)

The blue components are inputs active for the entire runtime of the CanSat, data from the red sensors will only be collected while the CanSat is in normal mode, and the purple components are outputs.