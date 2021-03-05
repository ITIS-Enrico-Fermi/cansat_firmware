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

FILE *log_stream;


typedef struct {
    int timestamp;
    EventBits_t contains;
    bme280_data_t ambient;
    gps_position_t position;
    struct sps30_measurement partmatter;
} Payload_t;


struct query_sensors_params {
    QueueHandle_t pipeline;
    EventGroupHandle_t dev_barrier;
    GPSDevice_t gps_dev;
    QueueHandle_t pm_queue;
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
            DEV_SPS30,//DEV_BME280 | DEV_GPS,
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

        if(ready && DEV_SPS30) {
            struct sps30_measurement partmatter;
            xQueueReceive(parameters->pm_queue, &partmatter, 100 / portTICK_RATE_MS);

            payload.partmatter = partmatter;
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
    char out_buf[1000];

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

        if(payload.contains && DEV_SPS30) {
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

    struct query_sensors_params *env = (struct query_sensors_params *)pvParameters;

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

    while(1) {
        sensirion_sleep_usec(SPS30_MEASUREMENT_DURATION_USEC);
        ret = sps30_read_measurement(&m);

        if(ret < 0) {
            ESP_LOGW(TAG, "SPS30 Cannot retrieve last measurement");
        } else {
            ESP_LOGI(TAG, "measured values:\n"
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
                   m.mc_1p0, m.mc_2p5, m.mc_4p0, m.mc_10p0, m.nc_0p5, m.nc_1p0,
                   m.nc_2p5, m.nc_4p0, m.nc_10p0, m.typical_particle_size);

            //xQueueSend(env->pm_queue, &m, 100 / portTICK_RATE_MS);
            //xEventGroupSetBits(env->dev_barrier, DEV_SPS30);
        }
    }

    vTaskDelete(NULL);
}

void app_main() {
    EventGroupHandle_t device_barrier = xEventGroupCreate();

    QueueHandle_t pipeline = xQueueCreate(10, sizeof(Payload_t));
    QueueHandle_t pm_queue = xQueueCreate(10, sizeof(struct sps30_measurement));

    ESP_LOGI(TAG, "Let's start our tasks");

    struct query_sensors_params qstask_params = {
        .pipeline       = pipeline,
        .dev_barrier    = device_barrier,
        .gps_dev        = NULL,
        .pm_queue       = pm_queue
    };
    xTaskCreate(query_sensors_task, "query_sensors", 4096, (void*)&qstask_params, 10, NULL);        //Query sensors
    xTaskCreate(prepare_payload_task, "prepare_payload", 4096, pipeline, 10, NULL);                 //Prepare payload for tx

    xTaskCreate(sps30_task, "sps30", 4096, (void*)&qstask_params, 10, NULL);

}