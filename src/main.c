#include <stdlib.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"

#define TAG "Main"

#include "gps.h"
#include "driver/uart.h"

#include "bme280_i2c.h"
#include "bme280.h"

void app_main() {
    bme280_i2c_init();
    bme280_setup(BME280_OVERSAMPLING_4X, BME280_OVERSAMPLING_1X, BME280_OVERSAMPLING_16X, BME280_FILTER_COEFF_8);
    xTaskCreate(bme280_task_normal_mode, "bme280", 2048, NULL, 10, NULL);

    bme280_register_handler(xTaskGetCurrentTaskHandle());
    bme280_set_delay(1000);

    gps_configure(UART_NUM_2);
    
    xTaskCreate(gps_location_update_handler, "GPS_location_task", 2048, NULL, 10, NULL);
    
    EventGroupHandle_t* gps_status = gps_get_status();
    
    while(xEventGroupWaitBits(*gps_status, GPS_FIXED_POSITION, pdFALSE, pdFALSE, pdMS_TO_TICKS(2000)) == pdFALSE);

    while(1) {

        if(xTaskNotifyWait(BME280_MEASURE_UPDATED, 0x00, NULL, pdMS_TO_TICKS(1000)) == pdTRUE) {
            bme280_data_t measure = bme280_get_last_measure();
            printf("T: %.2f, P: %.2f, h: %.2f\n", measure.temperature, measure.pressure, measure.humidity);
        }

        //Wait for notification update
        if( xTaskNotifyWait(GPS_LOCATION_UPDATED, 0x00, NULL, pdMS_TO_TICKS(1100)) == pdTRUE ) {
            gps_position_t current_position;
            if(gps_get_current_position(&current_position)) {
                printf("Latitude: %.6f, Longitude: %.6f, Altitude: %.6f, Fix quality: %d\n", minmea_tocoord(&current_position.latitude), minmea_tocoord(&current_position.longitude), minmea_tocoord(&current_position.altitude), current_position.fix_quality);
            }
        } else {
            ESP_LOGW(TAG, "Location was not updated within 1 second");
        }
        
    }
}