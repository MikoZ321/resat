# ReSat
This repository contains all of the code used by the ReSAT Team taking part in the 2024/2025 European CanSat Competition.

## Setup Process

1. This project was coded in Arduino IDE using the esp32 board library by Espressif (for a step-by-step setup process consult the link https://dronebotworkshop.com/esp32-intro/)

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