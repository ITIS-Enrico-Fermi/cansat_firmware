#include <stdlib.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include "esp_log.h"
#include "driver/gpio.h"

#define TAG "Main"

#include "spi/spi.h"
#include "sdcard.h"

#include "bme280.h"
#include "bme280_i2c.h"
#include "gps.h"

#include "driver/i2c.h"

#include "devices.h"

#include "sps30.h"

/*
FILE *log_stream;
struct sps30_measurement sps30_last_measure;
EventGroupHandle_t device_barrier;*/

typedef struct {
    int timestamp;
    EventBits_t contains;
    bme280_data_t ambient;
    gps_position_t position;
    struct sps30_measurement partmatter;
} Payload_t;

struct task_parameters {
    QueueHandle_t pipeline;
    EventGroupHandle_t dev_barrier;
    GPSDevice_t gps_dev;
    QueueHandle_t pm_queue;
};



struct task_parameters task_params;



void query_sensors_task(void *pvParameters) {

    struct task_parameters *tp = (struct task_parameters *)pvParameters;

    QueueHandle_t pipeline = tp->pipeline;
    EventGroupHandle_t devices_barrier = tp->dev_barrier;
    GPSDevice_t gps = tp->gps_dev;

    Payload_t payload;

    EventBits_t querying = DEV_BME280 + DEV_GPS + DEV_SPS30;

    while(true) {

        EventBits_t ready = xEventGroupWaitBits(
            devices_barrier,
            querying,
            pdTRUE,
            pdFALSE,
            1000
        );

        if(ready != 0) {

        if(ready & DEV_BME280) {
            bme280_data_t ambient = bme280_get_last_measure();

            payload.ambient = ambient;
        }
        
        if(ready & DEV_GPS) {
            gps_position_t position;
            gps_get_current_position(gps, &position);

            payload.position = position;
        }

        if(ready & DEV_SPS30) {
            struct sps30_measurement m;
            xQueueReceive(tp->pm_queue, &m, 1000 / portTICK_RATE_MS);

            payload.partmatter = m;
        }

        payload.contains = ready & querying;

        xQueueSend(pipeline, &payload, 100 / portTICK_PERIOD_MS);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


void prepare_payload_task(void *pvParameters) {

    struct task_parameters *tp = (struct task_parameters *)pvParameters;
    QueueHandle_t pipeline = tp->pipeline;

    Payload_t payload;
    char out_buf[1000];

    while(true) {
        
        if(xQueueReceive(pipeline, &payload, 1000 / portTICK_PERIOD_MS) == pdTRUE) {
            ESP_LOGI("payload_task", "Received payload");

        if(payload.contains & DEV_BME280) {
            bme280_data_t *amb = &payload.ambient;
            sprintf(out_buf, "T: %.2f, P: %.2f, h: %.2f", amb->temperature, amb->pressure, amb->humidity);
            printf("%s\n", out_buf);
            //fprintf(log_stream, "%s\t", out_buf);
        }

        if(payload.contains & DEV_GPS) {
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

        if(payload.contains & DEV_SPS30) {
            struct sps30_measurement *pm = &payload.partmatter;
            sprintf(
                out_buf,
                "measured values:\n"
                "\t%0.2f pm1.0\n"
                "\t%0.2f pm2.5\n"
                "\t%0.2f pm4.0\n"
                "\t%0.2f pm10.0\n"
                "\t%0.2f nc0.5\n"
                "\t%0.2f nc1.0\n"
                "\t%0.2f nc2.5\n"
                "\t%0.2f nc4.5\n"
                "\t%0.2f nc10.0\n"
                "\t%0.2f typical particle size\n",
                pm->mc_1p0, pm->mc_2p5, pm->mc_4p0, pm->mc_10p0, pm->nc_0p5, pm->nc_1p0,
                pm->nc_2p5, pm->nc_4p0, pm->nc_10p0, pm->typical_particle_size
            );
            printf("%s\n", out_buf);
        }

        //fflush(log_stream);

        }

    }
}

void sps30_task(void *pvParameters) {
    int16_t ret;
    struct sps30_measurement m;

    struct task_parameters *tp = (struct task_parameters *)pvParameters;

    EventGroupHandle_t devices_barrier = tp->dev_barrier;

    sensirion_i2c_select_bus(I2C_NUM_0);
    sensirion_i2c_init();

    while(sps30_probe() != 0) {
        ESP_LOGW(TAG, "SPS30 probing failed.");
        sensirion_sleep_usec(1000000);
    }
    ESP_LOGI(TAG, "SPS30 successful probing.");

    uint8_t fw_major, fw_minor;
    ret = sps30_read_firmware_version(&fw_major, &fw_minor);
    if(ret) {
        ESP_LOGW(TAG, "SPS30 error reading firmware");
    } else {
        ESP_LOGD(TAG, "SPS30 firmware version %u.%u", fw_major, fw_minor);
    }

    ret = sps30_start_measurement();
    if(ret)
        ESP_LOGE(TAG, "SPS30 Error starting measurement");
    ESP_LOGD(TAG, "SPS30 Started measurement");

    while(true) {
        sensirion_sleep_usec(SPS30_MEASUREMENT_DURATION_USEC);
        ret = sps30_read_measurement(&m);

        if(ret < 0) {
            ESP_LOGW(TAG, "SPS30 Cannot retrieve last measurement");
        } else {
            xQueueSend(tp->pm_queue, &m, 100 / portTICK_RATE_MS);
            xEventGroupSetBits(devices_barrier, DEV_SPS30);
        }
    }

    vTaskDelete(NULL);
}


void app_main() {

    task_params = (struct task_parameters){
        .pipeline       = xQueueCreate(10, sizeof(Payload_t)),
        .dev_barrier    = xEventGroupCreate(),
        .pm_queue       = xQueueCreate(10, sizeof(struct sps30_measurement))
    };

    xTaskCreate(sps30_task, "sps30", 2048, &task_params, 1, NULL);
    xTaskCreate(query_sensors_task, "query", 2048, &task_params, 1, NULL);
    xTaskCreate(prepare_payload_task, "payload", 4096, &task_params, 1, NULL);
    
}