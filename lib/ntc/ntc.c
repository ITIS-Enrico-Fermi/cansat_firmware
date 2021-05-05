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

// GPIO4 -> NTC circuit
// Using ADC 2 CH 0

/* tools */

// voltage at B (10kohm)
#define V_NOD   (553)
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

    // Configuration for ADC 1 CH 7
    // adc1_config_width(ADC_WIDTH_BIT_12);
    // adc1_config_channel_atten(ADC_CHANNEL_7, ADC_ATTEN_DB_11);

    // Configuration for ADC 2 CH 0
    adc2_config_channel_atten(ADC_CHANNEL_0, ADC_ATTEN_DB_11);


    //  Compute calibration curve
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_cal);

    initialized = true;

    return 0;
}

void ntc_task(void *pvParameters) {
    int adc_reading = 0;
    uint32_t voltage = 0;
    double temp;

    while(true) {
        // adc_reading = adc1_get_raw(ADC_CHANNEL_7);
        adc2_get_raw(ADC_CHANNEL_0, ADC_WIDTH_BIT_12, &adc_reading);
        voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_cal);

        temp = voltage_to_temperature(voltage);

        // ESP_LOGI(TAG, "Raw: %d, voltage: %dmV\n", adc_reading, voltage);

        // ESP_LOGI(TAG, "Temperature in degrees: %.2f\n", temp);

        xQueueSend(queue, &temp, 100 / portTICK_PERIOD_MS);
        xEventGroupSetBits(barrier, id);        

        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}