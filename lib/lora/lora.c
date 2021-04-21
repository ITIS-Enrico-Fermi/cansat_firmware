/**
 * @file lora.c
 * @author CanSat team
 * @date 21 Apr 2021
 * @brief LoRa wrapper for RFM95 (RFM9_) module
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <string.h>

#include "lora.h"

#define TAG "LORA"

static struct lora_shrinked_payload sp_local;
static SemaphoreHandle_t lock;
static TaskHandle_t task_to_notify;

void lora_setup(struct lora_cfg *cfg) {
    rfm95_init(&(cfg->spi));
    rfm95_set_frequency(cfg->freq);
    rfm95_set_tx_power(cfg->tp);
    rfm95_set_spreading_factor(cfg->sf);
    rfm95_set_bandwidth(cfg->bw);
    if (cfg->is_crc_en)
        rfm95_enable_crc();
    lock = xSemaphoreCreateBinary();
    xSemaphoreGive(lock);
    ESP_LOGD(TAG, "Setup finished\n");
}

void lora_shrink_payload(Payload_t *src, struct lora_shrinked_payload *dst) {
    dst->timestamp = src->timestamp;
    
    dst->bme280.p = src->ambient.pressure;
    dst->bme280.t = src->ambient.temperature;
    dst->bme280.h = src->ambient.humidity;

    dst->gps.alt = src->position.altitude.value;
    dst->gps.lon = src->position.longitude.value;
    dst->gps.lat = src->position.latitude.value;

    dst->sps30.mc_1p0 = src->partmatter.mc_1p0;
    dst->sps30.mc_2p5 = src->partmatter.mc_2p5;
    dst->sps30.mc_4p0 = src->partmatter.mc_4p0;
    dst->sps30.mc_10p0 = src->partmatter.mc_10p0;
    dst->sps30.nc_0p5 = src->partmatter.nc_0p5;
    dst->sps30.nc_1p0 = src->partmatter.nc_1p0;
    dst->sps30.nc_2p5 = src->partmatter.nc_2p5;
    dst->sps30.nc_4p0 = src->partmatter.nc_4p0;
    dst->sps30.nc_10p0 = src->partmatter.nc_10p0;
    dst->sps30.typical_particle_size = src->partmatter.typical_particle_size;
}

void lora_set_payload(struct lora_shrinked_payload *sp) {
    xSemaphoreTake(lock, 200/portTICK_PERIOD_MS);
    memcpy((void *)&sp_local, (void *)sp, sizeof(struct lora_shrinked_payload));
    xSemaphoreGive(lock);
}

void lora_shrink_and_set_payload(Payload_t *p) {
    struct lora_shrinked_payload sp;
    lora_shrink_payload(p, &sp);
    lora_set_payload(&sp);
}

void lora_send_payload(struct lora_shrinked_payload *sp) {
    BaseType_t task_woken = pdFALSE;
    lora_set_payload(sp);
    xTaskNotifyGive(task_to_notify);
}

void lora_send(Payload_t *p) {
    BaseType_t task_woken = pdFALSE;
    lora_shrink_and_set_payload(p);
    xTaskNotifyGive(task_to_notify);
}

void lora_trasmission_task(void *pv) {
    task_to_notify = xTaskGetCurrentTaskHandle();
    static uint32_t notification;
    for (;;) {
        notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (notification) {
            xSemaphoreTake(lock, 200/portTICK_PERIOD_MS);
            rfm95_send_packet((uint8_t *)&sp_local, sizeof(sp_local));
            xSemaphoreGive(lock);
        }
    }
}