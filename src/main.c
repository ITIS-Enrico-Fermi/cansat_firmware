/**
 * 
 * @file main.c
 * @author sCanSati Team 2020-2021, ITIS E. Fermi, Modena
 *      Bendin Andrea
 *      Biolchini Marco
 *      Corradini Giulio
 *      Gabbi Giulio
 *      Mecatti Francesco
 *      Muratori Alessandro
 *      
 *      Prof. Prandini Annamaria
 * 
 */

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
#include "adxl345.h"

#include "math.h"

#define LORA_ID (245)  // 65 90 90 to ASCII

EventBits_t sending = DEV_SD | DEV_RFM95;  // TODO: decomment in prod
// EventBits_t sending = DEV_RFM95;

//  Enabled devices/sensors (e.g. DEV_BME280 | DEV_GPS)
EventBits_t querying = DEV_NTC | DEV_SPS30 | DEV_BME280 | DEV_GPS | DEV_IMU;  // TODO: decomment in prod
// EventBits_t querying = DEV_NTC | DEV_BME280 | DEV_GPS;

EventBits_t recovery = DEV_BUZZ | DEV_FAN;  // TODO: decomment in prod
// EventBits_t recovery = 0;

struct task_parameters {
    QueueHandle_t pipeline;
    EventGroupHandle_t dev_barrier;
    GPSDevice_t gps_dev;
    QueueHandle_t pm_queue;
    QueueHandle_t ntc_queue;
    QueueHandle_t accel_queue;
    FILE *pretty_file;
    FILE *csv_file;
};
struct task_parameters task_params;

typedef enum {
    START = 0,
    INIT,  // Init sensors, fan, buzzer, measure pressure
    WAITING_LAUNCH,  // Wait until accel_z > 2
    LAUNCHED,  // Start pressure measurement every 5 secs
    ALTITUDE_OVER_500M,
    FALLING,  // Fan on
    ALTITUDE_SUB_200M,  // Fan off
    LANDED,  // Buzzer on
} State;

static State state = START;

static double current_alt, old_alt;
static struct accelerometer_data old_accel;
static int cont = 0;

static double p0 = 0.0;

static double alt_from_press(float press) {
    return (44330.0 * (1.0 - pow(press / p0, 0.19029495)));
}

static bool accel_equal(struct accelerometer_data *first, struct accelerometer_data *second) {
    return (first->x == second->x) && (first->y == second->y) && (first->z == second->z);
}

void finite_state_machine(Payload_t *p) {  // Called each second
    static struct task_parameters *tp = &task_params;
    switch (state) {
        case START:
            fprintf(tp->pretty_file, "START\n");
            if (cont >= 5)
                state = INIT;
            cont++;
            break;
        case INIT:
            fprintf(tp->pretty_file, "INIT\n");
            p0 = p->ambient.pressure;
            ESP_LOGD("fsm", "first_alt: %f", p0);
            state = WAITING_LAUNCH;
            break;
        case WAITING_LAUNCH:
            fprintf(tp->pretty_file, "WAITING_LAUNCH\n");
            ESP_LOGD("fsm", "waiting launch, accel: %d", p->accelerometer.z);
            if (p->accelerometer.z > 20) {
                state = LAUNCHED;
            }
            break;
        case LAUNCHED:
            fprintf(tp->pretty_file, "LAUNCHED\n");
            ESP_LOGD("fsm", "launched, delta press: %lf", alt_from_press(p->ambient.pressure));
            if (alt_from_press(p->ambient.pressure) > 500) {
                state = ALTITUDE_OVER_500M;
                old_alt = alt_from_press(p->ambient.pressure);
            }
            break;
        case ALTITUDE_OVER_500M:
            fprintf(tp->pretty_file, "ALTITUDE_OVER_500M\n");
            ESP_LOGD("fsm", "alt over 500m");
            current_alt = alt_from_press(p->ambient.pressure);
            if (old_alt > current_alt+1) {
                fan_set_speed(100);
                state = FALLING;
                old_alt = current_alt;
            }
            old_alt = current_alt;
            break;
        case FALLING:
            fprintf(tp->pretty_file, "FALLING\n");
            ESP_LOGD("fsm", "falling");
            if (alt_from_press(p->ambient.pressure) < 200) {
                fan_off();
                state = ALTITUDE_SUB_200M;
                old_accel = p->accelerometer;
            }
            break;
        case ALTITUDE_SUB_200M:
            fprintf(tp->pretty_file, "ALTITUDE_SUB_200M\n");
            ESP_LOGD("fsm", "sub 200m");
            if (accel_equal(&old_accel, &(p->accelerometer))) {
                xTaskCreate(buzzzer_beeper_task, "buzzer beeper", 2048, NULL, 10, NULL);
                state = LANDED;
            }
            old_accel = p->accelerometer;
            break;
        case LANDED:
            fprintf(tp->pretty_file, "LANDED\n");
            break;
        default:
            ESP_LOGD("fsm", "unknown state");
            state = LANDED;
            break;
    }
}

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

        if (ready & DEV_IMU) {
            struct accelerometer_data accel_data;
            xQueueReceive(tp->accel_queue, &accel_data, 1000 / portTICK_PERIOD_MS);

            payload.accelerometer = accel_data;
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

        if(payload.contains & DEV_IMU) {
            out_buf_len += sprintf(
                out_buf+out_buf_len,
                "Accelerometer:\n"
                "\tx: %d"
                "\ty: %d"
                "\tz: %d\n",
                payload.accelerometer.x,
                payload.accelerometer.y,
                payload.accelerometer.z
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

        finite_state_machine(&payload);
        
        if (sending & DEV_SD) {
            fprintf(tp->pretty_file, out_buf);  // The stream is buffered, no concerns about delay (?)
            fprintf(tp->pretty_file, "\n");
            fflush(tp->pretty_file);
            fsync(fileno(tp->pretty_file));  // Performance decreasing

            // fprintf(tp->csv_file, csv_buf);
            // fflush(tp->csv_file);
            // fsync(fileno(tp->csv_file));

            // TODO: Close both files somewhere
        }
        
        }

    }
}

void self_test(void *_pv) {
    ESP_LOGI("self_test", "self test done");
    for (int i=0; i<5; i++) {
        buzzer_on();
        vTaskDelay(500/portTICK_RATE_MS);
        buzzer_off();
        vTaskDelay(500/portTICK_RATE_MS);
    }

    fan_set_speed(100);
    vTaskDelay(1000/portTICK_PERIOD_MS);
    fan_set_speed(50);
    vTaskDelay(1000/portTICK_PERIOD_MS);
    fan_off();
    vTaskDelay(1000/portTICK_PERIOD_MS);
    ESP_LOGI("self_test", "self test done finito");
}

void on_launch(void *_pv) {
    fprintf(task_params.pretty_file, "LAUNCHED\n");
    fflush(task_params.pretty_file);
    fsync(fileno(task_params.pretty_file));  // Performance decreasing
}

void fan_on(void *_pv) {
    fan_set_speed(100);
}

void on_landing(void *_pv) {
    fprintf(task_params.pretty_file, "LANDING\n");
    fflush(task_params.pretty_file);
    fsync(fileno(task_params.pretty_file));  // Performance decreasing
}

void on_ground(void *_pv) {
    buzzer_on();
    fclose(task_params.pretty_file);
    fclose(task_params.csv_file);    
}

void app_main() {
       task_params = (struct task_parameters){
        .pipeline       = xQueueCreate(10, sizeof(Payload_t)),
        .dev_barrier    = xEventGroupCreate(),
        .pm_queue       = xQueueCreate(1, sizeof(struct sps30_measurement)),
        .ntc_queue      = xQueueCreate(1, sizeof(double)),
        .accel_queue    = xQueueCreate(1, sizeof(struct accelerometer_data)),
        .pretty_file    = NULL,
        .csv_file       = NULL
    };
    
    ESP_LOGI(TAG, "MAIN");
    ESP_LOGI("init_all", "init_all entered");
    struct task_parameters tp = task_params;
    if(querying & DEV_SPS30) {
        struct sps30_task_parameters sps30_params = {
            .dev_barrier = tp.dev_barrier,
            .pm_queue = tp.pm_queue,
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
            .sync_barrier = tp.dev_barrier,
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
            .sync_barrier = tp.dev_barrier,
            .sync_id = DEV_GPS,
            .uart = {
                .tx = 17,
                .rx = 16
            }
        };
        tp.gps_dev = gps_setup_new(&gps_config);
        xTaskCreate(gps_task, "gps", 2048, tp.gps_dev, 10, NULL);
    }

    if(querying & DEV_NTC) {
        struct ntc_config ntc_config = {
            .sync_barrier = tp.dev_barrier,
            .ntc_queue = tp.ntc_queue,
            .device_id = DEV_NTC,
            .adc_num = ADC_UNIT_2,
            .adc_ch = ADC_CHANNEL_0
        };
        //Configure ADC on pin D5(?)
        ntc_init(&ntc_config);

        xTaskCreate(ntc_task, "ntc", 2048, NULL, 10, NULL);
    }

    if (querying & DEV_IMU) {
        struct adxl345_i2c_conf conf = {
            .sda = 0,
            .scl = 15,
            .bus = I2C_NUM_1,
            .range = ADXL345_RANGE_16_G,

            .device_id = DEV_IMU,
            .sync_barrier = tp.dev_barrier,
            .data_queue = tp.accel_queue
        };
        adxl345_init(&conf);
        xTaskCreate(accelerometer_task, "adxl345", 2048, NULL, 10, NULL);
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
            .bw = 250e3,
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
        tp.pretty_file = sdcard_get_stream("msr.log");
        tp.csv_file    = sdcard_get_stream("msr.csv");
    }

    if (recovery & DEV_BUZZ) {
        buzzer_init(5);
        buzzer_off();
    }

    if (recovery & DEV_FAN) {
        struct fan_config conf = {
            .pwm_pin = 13,
            .tach_pin = 14
        };
        fan_init(&conf);

        xTaskCreate(fan_task, "fan_task", 2048, NULL, 10, NULL);
        xTaskCreate(fan_count_task, "fan_count_task", 2048, NULL, 10, NULL);
    }

    xTaskCreate(query_sensors_task, "query", 2048, &tp, 1, NULL);
    xTaskCreate(prepare_payload_task, "payload", 8192, &tp, 1, NULL);
    ESP_LOGD(TAG, "Config finished");
}