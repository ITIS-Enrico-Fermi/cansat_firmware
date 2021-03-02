#include <stdlib.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include "esp_log.h"
#include "driver/gpio.h"

#define TAG "Main"

#include "gps.h"
#include "driver/uart.h"

#include "i2c/i2c.h"
#include "bme280_i2c.h"
#include "bme280.h"

#include "spi/spi.h"
#include "sdcard.h"

#include "devices.h"

FILE *log_stream;


typedef struct {
    int timestamp;
    EventBits_t contains;
    bme280_data_t ambient;
    gps_position_t position;
} Payload_t;


struct query_sensors_params {
    QueueHandle_t pipeline;
    EventGroupHandle_t dev_barrier;
    GPSDevice_t gps_dev;
};
void query_sensors_task(void *params) {

    struct query_sensors_params *parameters = (struct query_sensors_params*)params;

    QueueHandle_t pipeline = parameters->pipeline;
    EventGroupHandle_t devices_barrier = parameters->dev_barrier;
    GPSDevice_t gps = parameters->gps_dev;

    Payload_t payload;

    while(true) {

        EventBits_t ready = xEventGroupWaitBits(
            devices_barrier,
            DEV_BME280 | DEV_GPS,
            pdTRUE,
            pdFALSE,
            1000
        );


        if(ready != 0) {

        if(ready && DEV_BME280) {
            bme280_data_t ambient = bme280_get_last_measure();

            payload.ambient = ambient;
        }
        
        if(ready && DEV_GPS) {
            gps_position_t position;
            gps_get_current_position(gps, &position);

            payload.position = position;
        }

        payload.contains = ready;

        xQueueSend(pipeline, &payload, 100 / portTICK_PERIOD_MS);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


void prepare_payload_task(void *params) {

    QueueHandle_t pipeline = (QueueHandle_t)params;

    Payload_t payload;
    char out_buf[150];

    while(true) {
        
        if(xQueueReceive(pipeline, &payload, 1000 / portTICK_PERIOD_MS) == pdTRUE) {
            ESP_LOGI("payload_task", "Received payload");

        if(payload.contains && DEV_BME280) {
            bme280_data_t *amb = &payload.ambient;
            sprintf(out_buf, "T: %.2f, P: %.2f, h: %.2f", amb->temperature, amb->pressure, amb->humidity);
            printf("%s\n", out_buf);
            //fprintf(log_stream, "%s\t", out_buf);
        }

        if(payload.contains && DEV_GPS) {
            gps_position_t *pos = &payload.position;
            sprintf(
                out_buf,
                "Latitude: %.6f, Longitude: %.6f, Altitude: %.6f, Fix quality: %d",
                minmea_tocoord(&pos->latitude),
                minmea_tocoord(&pos->longitude),
                minmea_tofloat(&pos->altitude),
                pos->fix_quality
            );
            printf("%s\n", out_buf);
            //fprintf(log_stream, "%s\n", out_buf);
        }

        //fflush(log_stream);

        }

    }
}


void app_main() {
    EventGroupHandle_t device_barrier = xEventGroupCreate();

    i2c_init();

    bme280_config_t bme280_config = {
        .t_os = BME280_OVERSAMPLING_4X,
        .p_os = BME280_OVERSAMPLING_1X,
        .h_os = BME280_OVERSAMPLING_16X,
        .filter_k = BME280_FILTER_COEFF_8,
        .parent_task = xTaskGetCurrentTaskHandle(),
        .delay = 1000,
        .sync_barrier = device_barrier,
        .sync_id = DEV_BME280,
    };
    bme280_setup(&bme280_config);
    xTaskCreate(bme280_task_normal_mode, "bme280", 2048, NULL, 10, NULL);


    EventGroupHandle_t gps_status = xEventGroupCreate();
    GPSConfig_t gps_config = (GPSConfig_t){
        .uart_controller_port = UART_NUM_2,
        .parent_task = xTaskGetCurrentTaskHandle(),
        .gps_status = gps_status,
        .sync_barrier = device_barrier,
        .sync_id = DEV_GPS
    };
    GPSDevice_t gps = gps_setup_new(&gps_config);
    xTaskCreate(gps_task, "GPS_location_task", 4096, (void*)gps, 10, NULL);

    //sdcard_init();

    //log_stream = fopen("/sdcard/cansat.txt", "a");

    //fprintf(log_stream, "Started writing\n");
    
    while(xEventGroupWaitBits(gps_status, GPS_FIXED_POSITION, pdFALSE, pdFALSE, pdMS_TO_TICKS(1000)) == pdFALSE);

    QueueHandle_t pipeline = xQueueCreate(10, sizeof(Payload_t));

    ESP_LOGI(TAG, "Let's start our tasks");

    struct query_sensors_params qstask_params = {
        .pipeline=pipeline,
        .dev_barrier=device_barrier,
        .gps_dev=gps
    };
    xTaskCreate(query_sensors_task, "query_sensors", 4096, (void*)&qstask_params, 10, NULL);        //Query sensors
    xTaskCreate(prepare_payload_task, "prepare_payload", 4096, pipeline, 10, NULL);                 //Prepare payload for tx

}