#include "ntc.h"
#include "esp_log.h"
#include "stdint.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static esp_adc_cal_characteristics_t adc_cal;
static bool initialized = false;

int ntc_init() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL_7, ADC_ATTEN_DB_11);

    //  Compute calibration curve
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_cal);

    initialized = true;

    return 0;
}

void ntc_task(void *pvParameters) {
    uint32_t adc_reading = 0;
    uint32_t voltage = 0;

    while(true) {
        adc_reading = adc1_get_raw(ADC_CHANNEL_7);
        voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_cal);

        printf("Raw: %d, voltage: %dmV\n", adc_reading, voltage);

        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}