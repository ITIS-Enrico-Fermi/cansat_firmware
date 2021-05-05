#include "sps30-query.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "driver/i2c.h"

#include "devices.h"

#define TAG "SPS30"

void sps30_task(void *pvParameters) {
    struct sps30_measurement m;

    struct sps30_task_parameters *tp = (struct sps30_task_parameters *)pvParameters;
    EventGroupHandle_t devices_barrier = (EventGroupHandle_t)tp->dev_barrier;
    QueueHandle_t pm_queue = (QueueHandle_t)tp->pm_queue;
    int sps30_id = (int)tp->device_id;

    #ifdef SPS30_I2C_ALONE
    sensirion_i2c_select_bus(I2C_NUM_1);
    sensirion_i2c_init();
    #else
    sensirion_i2c_select_bus(I2C_NUM_0);
    #endif

    while(sps30_probe() != 0) {
        ESP_LOGW(TAG, "Probing failed. Retrying...");
        sensirion_sleep_usec(1000000);
    }
    ESP_LOGI(TAG, "Successful probing");

    uint8_t fw_major, fw_minor;
    if(sps30_read_firmware_version(&fw_major, &fw_minor)) {
        ESP_LOGW(TAG, "Error reading firmware version");
    } else {
        ESP_LOGD(TAG, "Firmware version %u.%u", fw_major, fw_minor);
    }

    if(sps30_start_measurement()) {
        ESP_LOGE(TAG, "Can't start measurement");
        vTaskDelete(NULL);
    }
    ESP_LOGD(TAG, "Started measurement");

    while(true) {
        sensirion_sleep_usec(SPS30_MEASUREMENT_DURATION_USEC);

        if(sps30_read_measurement(&m) < 0) {
            ESP_LOGW(TAG, "SPS30 Cannot retrieve last measurement");
        } else {
            ESP_LOGD(TAG,
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
                m.mc_1p0, m.mc_2p5, m.mc_4p0, m.mc_10p0, m.nc_0p5, m.nc_1p0,
                m.nc_2p5, m.nc_4p0, m.nc_10p0, m.typical_particle_size
            );
            xQueueSend(pm_queue, &m, 100 / portTICK_RATE_MS);
            ESP_LOGD(TAG, "id: %d", sps30_id);
            xEventGroupSetBits(devices_barrier, sps30_id);
        }
    }

    vTaskDelete(NULL);
}