#include "ntc.h"
#include "esp_log.h"
#include "stdint.h"
#include "math.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static esp_adc_cal_characteristics_t adc_cal;
static bool initialized = false;

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

        printf("Temperature in degrees: %.2f\n", voltage_to_temperature(voltage));

        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}