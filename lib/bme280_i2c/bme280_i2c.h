#include "bme280.h"

#ifndef BME280_I2C_H__
#define BME280_I2C_H__

#ifndef BME280_I2C_ADDRESS
#define BME280_I2C_ADDRESS BME280_I2C_ADDRESS1
#endif

typedef struct {
    double temperature;
    double pressure;
    double humidity;
} bme280_measure;

//Configuration
void bme280_i2c_init(); //Enable I2C controller
s32 bme280_setup(u8 t_os, u8 p_os, u8 h_os, u8 filter_k);    //BME280_OVERSAMP_nX as params and BME280_FILTER_COEFF_n for filter coefficient


void bme280_read_normal();//s32, bme280_measure*);

//  FreeRTOS tasks
void task_bme280_normal_mode();
bme280_measure bme280_get_last_measure();
//void task_bme280_forced_mode(void *ignore);

 
#endif
