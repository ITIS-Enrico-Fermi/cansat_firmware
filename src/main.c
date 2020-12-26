#include <stdlib.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include "esp_log.h"

#define TAG "Main"

#include "gps.h"
#include "driver/uart.h"

#include "i2c/i2c.h"
#include "bme280_i2c.h"
#include "bme280.h"

#include "spi/spi.h"
#include "sdcard.h"

void app_main() {
    EventGroupHandle_t dev_consumers_sync_barrier = xEventGroupCreate();

    i2c_init();

    bme280_config_t bme280_config = {
        .t_os = BME280_OVERSAMPLING_4X,
        .p_os = BME280_OVERSAMPLING_1X,
        .h_os = BME280_OVERSAMPLING_16X,
        .filter_k = BME280_FILTER_COEFF_8,
        .parent_task = xTaskGetCurrentTaskHandle(),
        .sync_barrier = dev_consumers_sync_barrier,
        .delay = 1000,
    };
    bme280_setup(&bme280_config);
    xTaskCreate(bme280_task_normal_mode, "bme280", 2048, NULL, 10, NULL);


    EventGroupHandle_t gps_status = xEventGroupCreate();
    GPSConfig_t gps_config = (GPSConfig_t){
        .uart_controller_port = UART_NUM_2,
        .parent_task = xTaskGetCurrentTaskHandle(),
        .sync_barrier = dev_consumers_sync_barrier,
        .gps_status = gps_status
    };
    gps_setup(&gps_config);
    xTaskCreate(gps_task, "GPS_location_task", 4096, NULL, 10, NULL);

    //spi_init();
    sdcard_init();
    
    
    while(xEventGroupWaitBits(gps_status, GPS_FIXED_POSITION, pdFALSE, pdFALSE, pdMS_TO_TICKS(2000)) == pdFALSE);

    while(true) {

        //while(xEventGroupWaitBits(global_status, GPS_LOCATION_UPDATED, BME280_MEASURE_UPDATED, pdTRUE, pdTRUE, pdMS_TO_TICKS(1000)) == pdFALSE)

        if(xTaskNotifyWait(BME280_MEASURE_UPDATED, 0x00, NULL, pdMS_TO_TICKS(1000)) == pdTRUE) {
            bme280_data_t measure = bme280_get_last_measure();
            printf("T: %.2f, P: %.2f, h: %.2f\n", measure.temperature, measure.pressure, measure.humidity);
        } else {
            ESP_LOGW(TAG, "BME280 data was not updated within 1 second");
        }

        //Wait for notification update
        if( xTaskNotifyWait(GPS_LOCATION_UPDATED, 0x00, NULL, pdMS_TO_TICKS(1000)) == pdTRUE ) {
            gps_position_t current_position;
            if(gps_get_current_position(&current_position)) {
                printf("Latitude: %.6f, Longitude: %.6f, Altitude: %.6f, Fix quality: %d\n", minmea_tocoord(&current_position.latitude), minmea_tocoord(&current_position.longitude), minmea_tocoord(&current_position.altitude), current_position.fix_quality);
            }
        } else {
            ESP_LOGW(TAG, "Location was not updated within 1 second");
        }
        
    }
}