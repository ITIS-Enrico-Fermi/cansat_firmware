# Cansat Firmware

<p align="center">
  <img width="70%" src="docs/logo.png" />
</p>

_Awarded Highest Technical Achievement at European Cansat Competition 2021_

## 1. Overview

This firmware is tailored for the ESP32 microcontroller: the core component of the CanSat.
It’s the main accountable for the communication with every sensor, and it also handles
the transmission and logging.

Every peripheral is connected using its peculiar bus and wire protocol, thus drivers were
written for each device connected to the ESP32.

This repository only contains the firmware, for more details about the project visit
[ITIS Fermi | CanSat 2021](https://www.fermi-mo.edu.it/pagine/cansat-2021),
and check out gathered data from the National Flight Competition at
[Github | CanSatData](https://github.com/ITIS-Enrico-Fermi/CanSatData)

### 1.a. Devices

The CanSat is equipped with the following devices, mounted onto two circular and custom-designed
PCBs:

- HopeRF RFM96 LoRa transmission module

- Micro SD card

- Sensirion SPS30 PM sensor

- Bosch Sensortech BME280

- Noctua A4x10 PWM Fan with RPM measurement and PWM control

- u.blox NEO-6M GPS module

- ADXL345 three-axis accelerometer

- NTC G10K3976A1 with current mirror (connected to integrated ADC)

## 2. Software architecture

To meet these strict timing requirements, the firmware is based on a real time operating system:
FreeRTOS.

During the National Campaign flight there were issues with the I2C bus of the temperature sensor
and the accelerometer; those sensors stopped communicate data for more than a minute,
but the CanSat continued to transmit data from the other sensors.

## 3. Usage

Download the repository and compile the project using PlatformIO.

```
git clone https://github.com/ITIS-Enrico-Fermi/cansat_firmware
git submodule update --init

pio run build
```
