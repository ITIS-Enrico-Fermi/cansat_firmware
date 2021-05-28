#include "ntc.h"
#include "esp_log.h"
#include "stdint.h"
#include "math.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/queue.h>
#include <freertos/event_groups.h>

#define TAG "NTC"

static esp_adc_cal_characteristics_t adc_cal;
static bool initialized = false;
static QueueHandle_t queue;
static EventGroupHandle_t barrier;
static int id;
static int adc_num;
static int adc_ch;

// GPIO4 -> NTC circuit
// Using ADC 2 CH 0

/* tools */

// voltage at B (10kohm)
#define V_NOD   (432)
// temperature at B (25°C in K)
#define T_NOD   (298.15)
#define B       (3976)

/*
 *  @return temperature in °C
 */
double voltage_to_temperature(uint32_t voltage) {
    double t;

    double ntc_voltage = 3300 - voltage;
    t = (double)(T_NOD * B) / (T_NOD * log(ntc_voltage / V_NOD) + B) - 273.15;

    return (double)t;
}

/* public interface */

int ntc_init(struct ntc_config *config) {

    queue = (QueueHandle_t) config->ntc_queue;
    barrier = (EventGroupHandle_t) config->sync_barrier;
    id = (int) config->device_id;
    // Be aware: this code is meant to work with ADC 2
    adc_num = (int)config->adc_num;
    adc_ch = (int)config->adc_ch;

    if (adc_num == ADC_UNIT_1) {
        // Configuration for ADC 1 CH x
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(adc_ch, ADC_ATTEN_DB_11);
    } else if (adc_num == ADC_UNIT_2) {
        // Configuration for ADC 2 CH x
        adc2_config_channel_atten(adc_ch, ADC_ATTEN_DB_11);
    }

    //  Compute calibration curve
    // Get VRef calibration by running: espefuse.py --port /dev/ttyUSB0 adc_info
    esp_adc_cal_characterize(adc_num, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_cal);

    initialized = true;

    return 0;
}

void ntc_task(void *pvParameters) {
    int adc_reading = 0;
    uint32_t voltage = 0;
    double temp;

    while(true) {

        if (adc_num == ADC_UNIT_2) {
            adc2_get_raw(adc_ch, ADC_WIDTH_BIT_12, &adc_reading);
        } else if (adc_num == ADC_UNIT_1) {
            adc_reading = adc1_get_raw(adc_ch);
            // esp_adc_cal_get_voltage(ADC_CHANNEL_0, &adc_cal, &voltage);
        }

        voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_cal);
        // ESP_LOGI(TAG, "Voltage: %d", voltage);
        temp = voltage_to_temperature(voltage);

        // ESP_LOGI(TAG, "Raw: %d, voltage: %dmV\n", adc_reading, voltage);

        // ESP_LOGI(TAG, "Temperature in degrees: %.2f\n", temp);

        xQueueSend(queue, &temp, 100 / portTICK_PERIOD_MS);
        xEventGroupSetBits(barrier, id);        

        vTaskDelay(400 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}