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

void i2c_init();

#ifdef __cplusplus
}
#endif

#endif /* __busmanager_i2c */