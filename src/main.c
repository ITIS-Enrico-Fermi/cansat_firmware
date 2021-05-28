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
#include "esp_vfs.h"

#define TAG "MAIN"

#include "spi/spi.h"

#include "sdcard.h"
#include "csv_format.h"

#include "bme280.h"
#include "bme280_i2c.h"
#include "gps.h"

#include "driver/i2c.h"

#include "devices.h"

#include "sps30.h"
#include "sps30-query.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "ntc.h"

#include "lora.h"

#include "buzzer.h"

#include "nfa4x10.h"

#include "hook_manager.h"

#define LORA_ID (295)

EventBits_t sending = DEV_SD | DEV_RFM95;  // TODO: decomment in prod
// EventBits_t sending = DEV_RFM95;

//  Enabled devices/sensors (e.g. DEV_BME280 | DEV_GPS)
EventBits_t querying = DEV_NTC | DEV_SPS30 | DEV_BME280 | DEV_GPS;  // TODO: decomment in prod
// EventBits_t querying = DEV_NTC | DEV_BME280 | DEV_GPS;

EventBits_t recovery = DEV_BUZZ | DEV_FAN;  // TODO: decomment in prod
// EventBits_t recovery = 0;

struct task_parameters {
    QueueHandle_t pipeline;
    EventGroupHandle_t dev_barrier;
    GPSDevice_t gps_dev;
    QueueHandle_t pm_queue;
    QueueHandle_t ntc_queue;
    FILE *pretty_file;
    FILE *csv_file;
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
            pdMS_TO_TICKS(1000)
        );

        if(ready != 0) {
        
        payload.id = LORA_ID;

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
    char csv_buf[1024];
    int csv_buf_len;

    csv_heading(csv_buf,
                "time",
                querying & DEV_GPS ? "gps" : "",
                querying & DEV_BME280 ? "bme280" : "",
                querying & DEV_SPS30 ? "sps30" : "",
                querying & DEV_NTC ? "ntc" : "",
                NULL
                );  // TODO: Complete lib and data output in CSV format

    while(true) {
        
        if(xQueueReceive(pipeline, &payload, 1000 / portTICK_PERIOD_MS) == pdTRUE) {
        
        ESP_LOGI("payload_task", "Received payload");
        out_buf_len = 0;
        csv_buf_len = 0;

        time(&payload.timestamp);  // Offset between CanSat clock and BS start time
        out_buf_len += sprintf(out_buf+out_buf_len, "Time offset: %ld\n", payload.timestamp);
        
        if(payload.contains & DEV_BME280) {
            bme280_data_t *amb = &payload.ambient;
            out_buf_len += sprintf(out_buf+out_buf_len, "T: %.2f, P: %.2f, h: %.2f\n", amb->temperature, amb->pressure, amb->humidity);
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
                "NTC temperature: %.2f\n",
                payload.ntc_temp
            );
        }

        if (recovery & DEV_FAN) {
            out_buf_len += sprintf(
                out_buf+out_buf_len,
                "Fan speed: %d\n",
                fan_get_speed()
            );
        }

        ESP_LOGI(TAG, "%s\n", out_buf);

        if(sending & DEV_RFM95) {  // Send payload through LoRa point to point connection
            ESP_LOGD(TAG, "Sending payload...");
            lora_send(&payload);
        }
        
        if (sending & DEV_SD) {
            fprintf(tp->pretty_file, out_buf);  // The stream is buffered, no concerns about delay (?)
            fprintf(tp->pretty_file, "\n");
            fflush(tp->pretty_file);
            fsync(fileno(tp->pretty_file));  // Performance decreasing

            fprintf(tp->csv_file, csv_buf);
            fflush(tp->csv_file);
            fsync(fileno(tp->csv_file));

            // TODO: Close both files somewhere
        }

        }

    }
}

void app_main() {
    task_params = (struct task_parameters){
        .pipeline       = xQueueCreate(10, sizeof(Payload_t)),
        .dev_barrier    = xEventGroupCreate(),
        .pm_queue       = xQueueCreate(10, sizeof(struct sps30_measurement)),
        .ntc_queue      = xQueueCreate(10, sizeof(double)),
        .pretty_file    = NULL,
        .csv_file       = NULL
    };

    if(querying & DEV_SPS30) {
        struct sps30_task_parameters sps30_params = {
            .dev_barrier = task_params.dev_barrier,
            .pm_queue = task_params.pm_queue,
            .device_id = DEV_SPS30,
            .i2c_bus = I2C_NUM_0,
            .sda = 21,
            .scl = 22
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
                .sda = 0,
                .scl = 15,
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
            .device_id = DEV_NTC,
            .adc_num = ADC_UNIT_2,
            .adc_ch = ADC_CHANNEL_0
        };
        //Configure ADC on pin D5(?)
        ntc_init(&ntc_config);

        xTaskCreate(ntc_task, "ntc", 2048, NULL, 10, NULL);
    }

    if(sending & DEV_RFM95) {
        struct lora_cfg cfg = {
            .spi = {
                .cs = 2,
                .rst = 26,
                .miso = 19,
                .mosi = 23,
                .sck = 18
            },
            .sf = 9,
            .bw = 500e3,
            .tp = 17,
            .freq = 433e6,
            .is_crc_en = true,
        };
        lora_setup(&cfg);
        xTaskCreate(lora_transmission_task, "tx_task", 4096, NULL, 10, NULL);
    }

    if (sending & DEV_SD) {
        struct sdcard_config conf = {
            .spi = {
                .miso = 33,
                .mosi = 32,
                .sck = 25,
                .cs = 27
            },
            .format_sd_if_mount_failed = true,
            .max_files = 2
        };
        sdcard_init(&conf);
        task_params.pretty_file = sdcard_get_stream("msr.log");
        task_params.csv_file    = sdcard_get_stream("msr.csv");
    }

    if (recovery & DEV_BUZZ) {
        buzzer_init(5);
        // test

        // TODO: Decomment in prod
        // for (int i=0; i<5; i++) {
        //     buzzer_on();
        //     vTaskDelay(1000/portTICK_RATE_MS);
        //     buzzer_off();
        //     vTaskDelay(1000/portTICK_RATE_MS);
        // }
    }

    if (recovery & DEV_FAN) {
        struct fan_config conf = {
            .pwm_pin = 13,
            .tach_pin = 14
        };
        fan_init(&conf);

        xTaskCreate(fan_task, "fan_task", 2048, NULL, 10, NULL);
        xTaskCreate(fan_count_task, "fan_count_task", 2048, NULL, 10, NULL);

        // fan test
        fan_set_speed(100);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        // TODO: check if error with fan (blocked)
        // if (fan_get_speed() < 50) {
        // error = 1;
        // // Buzzer must make some beep   
        // }

        // TODO: Decomment
        // fan_set_speed(50);
        // vTaskDelay(1000/portTICK_PERIOD_MS);
        // fan_set_speed(25);
        // vTaskDelay(1000/portTICK_PERIOD_MS);
        // fan_off();
        // vTaskDelay(1000/portTICK_PERIOD_MS);
        fan_off();
    }

    xTaskCreate(query_sensors_task, "query", 2048, &task_params, 1, NULL);
    xTaskCreate(prepare_payload_task, "payload", 8192, &task_params, 1, NULL);
    ESP_LOGD(TAG, "Config finished");
}