# Cansat Firmware

This is firmware is built upon ESP-IDF Framework running on PlatformIO.
It is designed to work on ESP32 microcontroller.

So far, this feature were implemented:

* **GPS Library** can decode NMEA strings over UART controller 1 (pins TX:17 and RX:16), uses minmea.h library to convert strings to numbers. Requires FreeRTOS. Fully managed, exposes an API.
* **Temperature, humidity, pressure** using Bosch Sensortech BME280. Exposes an API. Requires FreeRTOS.