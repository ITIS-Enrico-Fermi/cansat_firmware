#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include "esp_log.h"
#include "driver/gpio.h"

#define TAG "MAIN"

#include "spi/spi.h"
#include "sdcard.h"

#include "bme280.h"
#include "bme280_i2c.h"
#include "gps.h"

#include "driver/i2c.h"

#include "devices.h"

#include "sps30.h"
#include "sps30-query.h"

#include "i2c/i2c.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "ntc.h"

#include "lora.h"

EventBits_t sending = DEV_RFM95 | DEV_SD;

//  Enabled devices/sensors (e.g. DEV_BME280 | DEV_GPS)
EventBits_t querying = DEV_NTC | DEV_SPS30 | DEV_BME280 | DEV_GPS;


//FILE *log_stream;

struct task_parameters {
    QueueHandle_t pipeline;
    EventGroupHandle_t dev_barrier;
    GPSDevice_t gps_dev;
    QueueHandle_t pm_queue;
    QueueHandle_t ntc_queue;
};
struct task_parameters task_params;



void query_sensors_task(void *pvParameters) {

    struct task_parameters *tp = (struct task_parameters *)pvParameters;

    QueueHandle_t pipeline = tp->pipeline;
    EventGroupHandle_t devices_barrier = tp->dev_barrier;
    GPSDevice_t gps = tp->gps_dev;

    Payload_t payload;

    while(true) {

        EventBits_t ready = xEventGroupWaitBits(
            devices_barrier,
            querying,
            pdTRUE,
            pdTRUE,  // pdTRUE in production (?)
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

        if(ready & DEV_NTC) {
            double ntc_temp;
            xQueueReceive(tp->ntc_queue, &ntc_temp, 1000 / portTICK_PERIOD_MS);

            payload.ntc_temp = ntc_temp;
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
    char out_buf[1024];
    int out_buf_len;

    while(true) {
        
        if(xQueueReceive(pipeline, &payload, 1000 / portTICK_PERIOD_MS) == pdTRUE) {
        
        ESP_LOGI("payload_task", "Received payload");
        out_buf_len = 0;

        time(&payload.timestamp);  // Offset between CanSat clock and BS start time
        out_buf_len += sprintf(out_buf+out_buf_len, "Time offset: %ld\n", payload.timestamp);
        
        if(payload.contains & DEV_BME280) {
            bme280_data_t *amb = &payload.ambient;
            out_buf_len += sprintf(out_buf+out_buf_len, "T: %.2f, P: %.2f, h: %.2f\n", amb->temperature, amb->pressure, amb->humidity);
            //fprintf(log_stream, "%s\t", out_buf);
        }

        if(payload.contains & DEV_GPS) {
            gps_position_t *pos = &payload.position;
            out_buf_len += sprintf(
                out_buf+out_buf_len,
                "Latitude: %.6f, Longitude: %.6f, Altitude: %.6f, Fix quality: %d\n",
                minmea_tocoord(&pos->latitude),
                minmea_tocoord(&pos->longitude),
                minmea_tofloat(&pos->altitude),
                pos->fix_quality
            );
            //fprintf(log_stream, "%s\n", out_buf);
        }

        if(payload.contains & DEV_SPS30) {
            struct sps30_measurement *pm = &payload.partmatter;
            out_buf_len += sprintf(
                out_buf+out_buf_len,
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
        }

        if(payload.contains & DEV_NTC) {
            out_buf_len += sprintf(
                out_buf+out_buf_len,
                "NTC temperature: %.2f",
                payload.ntc_temp
            );
        }

        if(sending & DEV_RFM95) {  // Send payload through LoRa point to point connection
            ESP_LOGI(TAG, "%s\n", out_buf);

            ESP_LOGD(TAG, "Sending payload...");
            lora_send(&payload);
        }

        //fflush(log_stream);

        }

    }
}

void app_main() {

    task_params = (struct task_parameters){
        .pipeline       = xQueueCreate(10, sizeof(Payload_t)),
        .dev_barrier    = xEventGroupCreate(),
        .pm_queue       = xQueueCreate(10, sizeof(struct sps30_measurement)),
        .ntc_queue      = xQueueCreate(10, sizeof(double))
    };

    if(querying & DEV_SPS30) {
        struct sps30_task_parameters sps30_params = {
            .dev_barrier = task_params.dev_barrier,
            .pm_queue = task_params.pm_queue,
            .device_id = DEV_SPS30,
            .i2c_bus = I2C_NUM_0
        };
        sps30_setup(&sps30_params);
        xTaskCreate(sps30_task, "sps30", 4096, NULL, 1, NULL);
    }

    if(querying & DEV_BME280) {
        bme280_config_t bme280_config = {
            .t_os = BME280_OVERSAMPLING_4X,
            .p_os = BME280_OVERSAMPLING_1X,
            .h_os = BME280_OVERSAMPLING_16X,
            .filter_k = BME280_FILTER_COEFF_8,
            .parent_task = xTaskGetCurrentTaskHandle(),
            .delay = 1000,
            .sync_barrier = task_params.dev_barrier,
            .sync_id = DEV_BME280,
            .i2c = {
                .sda = GPIO_NUM_5,
                .scl = GPIO_NUM_0,
                .bus = I2C_NUM_1
            }
        };
        bme280_setup(&bme280_config);
        xTaskCreate(bme280_task_normal_mode, "bme280", 2048, NULL, 10, NULL);
    }

    if(querying & DEV_GPS) {
        GPSConfig_t gps_config = {
            .uart_controller_port = 2,
            .gps_status = xEventGroupCreate(),
            .parent_task = xTaskGetCurrentTaskHandle(),
            .sync_barrier = task_params.dev_barrier,
            .sync_id = DEV_GPS,
            .uart = {
                .tx = 17,
                .rx = 16
            }
        };
        task_params.gps_dev = gps_setup_new(&gps_config);
        xTaskCreate(gps_task, "gps", 2048, task_params.gps_dev, 10, NULL);
    }

    if(querying & DEV_NTC) {
        struct ntc_config ntc_config = {
            .sync_barrier = task_params.dev_barrier,
            .ntc_queue = task_params.ntc_queue,
            .device_id = DEV_NTC
        };
        //Configure ADC on pin D5(?)
        ntc_init(&ntc_config);

        xTaskCreate(ntc_task, "ntc", 2048, NULL, 10, NULL);
    }

    if(sending & DEV_RFM95) {
        struct lora_cfg cfg = {
            .spi = {
                .cs = 15,
                .rst = 2,
                .miso = 19,
                .mosi = 23,
                .sck = 18
            },
            .freq = 868e6,
            .tp = 17,
            .sf = 12,
            .bw = 500000,
            .is_crc_en = true,
        };
        lora_setup(&cfg);
        xTaskCreate(lora_transmission_task, "tx_task", 4096, NULL, 10, NULL);
    }

    xTaskCreate(query_sensors_task, "query", 2048, &task_params, 1, NULL);
    xTaskCreate(prepare_payload_task, "payload", 4096, &task_params, 1, NULL);
    ESP_LOGD(TAG, "Config finished");
}