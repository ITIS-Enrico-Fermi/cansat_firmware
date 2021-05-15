#include "sps30-query.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "driver/i2c.h"

#include "devices.h"

#define TAG "SPS30"

static EventGroupHandle_t devices_barrier;
static QueueHandle_t pm_queue;
static int sps30_id;
static int i2c_bus;

void sps30_setup(struct sps30_task_parameters *params) {
    devices_barrier = params->dev_barrier;
    pm_queue = params->pm_queue;
    sps30_id = params->device_id;
    i2c_bus = params->i2c_bus;

    ESP_LOGD(TAG, "i2c bus: %d\nsps30_id: %d\n", i2c_bus, sps30_id);
    sensirion_i2c_select_bus(i2c_bus);
    sensirion_i2c_init();
}

void sps30_task(void *pvParameters) {
    struct sps30_measurement m;

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
            xQueueSend(pm_queue, &m, 100 / portTICK_RATE_MS);
            ESP_LOGD(TAG, "id: %d", sps30_id);
            xEventGroupSetBits(devices_barrier, sps30_id);
        }
    }

    vTaskDelete(NULL);
}