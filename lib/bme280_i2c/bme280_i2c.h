/*
Connection BME280 ----------> ESP32
            SCL   ----------> PIN 22
            SDA   ----------> PIN 21
*/

#ifndef BME280_I2C_H__
#define BME280_I2C_H__

#include "bme280.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct bme280_data bme280_data_t;
typedef struct {
    uint8_t t_os;
    uint8_t p_os;
    uint8_t h_os;
    uint8_t filter_k;
    TaskHandle_t parent_task;
    EventGroupHandle_t sync_barrier;
    int delay;
} bme280_config_t;

//Configuration
void bme280_i2c_init(); //Enable I2C controller
void bme280_setup(bme280_config_t *config);    //BME280_OVERSAMP_nX as params and BME280_FILTER_COEFF_n for filter coefficient
void bme280_set_delay(uint32_t delay_ms);

//  FreeRTOS tasks
void bme280_task_normal_mode(void* pv_params);
bme280_data_t bme280_get_last_measure();
//void task_bme280_forced_mode(void *ignore);

#define BME280_MEASURE_UPDATED 0x01
//void bme280_register_handler(TaskHandle_t handler);
 
#endif