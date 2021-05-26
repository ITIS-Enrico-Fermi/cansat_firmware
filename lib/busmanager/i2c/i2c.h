/*
 *  i2c.h
 * 
 *  by Giulio Corradini
 *  Cansat Firmware 2021
 * 
 *  Part of the bus manager HAL. Handles config of I2C bus
 *  for connected peripherals: BME280, IMU unit.
 * 
 */

#ifndef __busmanager_i2c
#define __busmanager_i2c

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

extern SemaphoreHandle_t i2c_mutex;
#define I2C_MUTEX(func) xSemaphoreTake(i2c_mutex, 1000 / portTICK_RATE_MS);\
                        func;\
                        xSemaphoreGive(i2c_mutex);

void i2c_init();

#ifdef __cplusplus
}
#endif

#endif /* __busmanager_i2c */