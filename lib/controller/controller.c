/**
 * @file controller.c
 * @author sCanSati Team 2020-2021, ITIS E. Fermi, Modena
 * @date 28 May 2021
 * @brief FSM implementation for launch sequence management.
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include "esp_log.h"

#include "controller.h"
#include "hook_manager.h"
#include "adxl345.h"

static QueueHandle_t payload_queue;
static Manager hm;
static double a0 = -1;
static const int d = 1.225;
static const int g = 9.80665;
static const char* tag = "controller";

void controller_init(Manager *m) {
    payload_queue = xQueueCreate(1, sizeof(Payload_t));
    hm = *m;
}

void controller_send_measurements(Payload_t *payload) {
    Payload_t p = *payload;
    ESP_LOGI(tag, "LOG");
    xQueueSend(payload_queue, &p, 10 / portTICK_RATE_MS);
}

static double alt_from_press(double p) {
    if (a0 == -1) {
        return p/(g*d);
    } else {
        return (p/(g*d)) - a0;
    }
}

void controller_task(void *pv) {
    State state = START;
    Payload_t payload;
    int i = 0;
    double current_alt, old_alt;
    struct accelerometer_data old_accel;

    for (;;) {
        bool has_payload = xQueueReceive(payload_queue, &payload, 10 / portTICK_RATE_MS);

        ESP_LOGI(tag, "entered loop");
        switch (state) {
            case START:
                state = INIT;
                ESP_LOGI(tag, "start");
            case INIT:
                ESP_LOGI(tag, "init");
                hm.invoke(ON_POWER_ON, NULL);
                state = GET_ALT;
            case GET_ALT:
                ESP_LOGI(tag, "get alt");
                a0 = alt_from_press(payload.ambient.pressure);
                ESP_LOGI("fsm", "a0: %lf", a0);
                state = WAITING_LAUNCH;
                break;
            case WAITING_LAUNCH:
                ESP_LOGI(tag, "WAITING_LAUNCH");
                if (has_payload && payload.accelerometer.z <= 2) {
                    hm.invoke(ON_LAUNCH, NULL);
                    state = LAUNCHED;
                }
                break;
            case LAUNCHED:
                ESP_LOGI(tag, "LAUNCHED");
                if (has_payload && alt_from_press(payload.ambient.pressure) > 500) {
                    state = ALTITUDE_OVER_500M;
                    old_alt = alt_from_press(payload.ambient.pressure);
                }
                break;
            case ALTITUDE_OVER_500M:
                ESP_LOGI(tag, "ALTITUDE_OVER_500M");
                if (has_payload) {
                    current_alt = alt_from_press(payload.ambient.pressure);
                    if (current_alt < old_alt) {
                        hm.invoke(ON_MAX_HEIGHT, NULL);
                        state = FALLING;
                    }
                    old_alt = current_alt;
                }
                break;
            case FALLING:
                ESP_LOGI(tag, "FALLING");
                if (has_payload && alt_from_press(payload.ambient.pressure) < 200) {
                    state = ALTITUDE_SUB_200M;
                    hm.invoke(ON_NEAR_GROUND, NULL);
                    old_accel = payload.accelerometer;
                }
                break;
            case ALTITUDE_SUB_200M:
                ESP_LOGI(tag, "ALTITUDE_SUB_200M");
                if (
                    has_payload &&
                    old_accel.x == payload.accelerometer.x &&
                    old_accel.y == payload.accelerometer.y &&
                    old_accel.z == payload.accelerometer.z
                    ) {
                        state = LANDED;
                }
                old_accel = payload.accelerometer;
                state = LANDED;
                break;
            case LANDED:
                hm.invoke(ON_GROUND, NULL);
                state = END;
                ESP_LOGI(tag, "LANDED");
                break;
            case END:
                break;
            default:
                state = LANDED;
                break;
        }
    }
}