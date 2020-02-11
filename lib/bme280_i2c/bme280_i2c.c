#include <esp_log.h>
#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include "bme280.h"
#include "bme280_i2c.h"

#define TAG  "BME280 I2C"

static s32 com_rstl;
static bme280_measure last_measure;
static SemaphoreHandle_t measure_sem;

void BME280_delay_msec(u32 msec)
{
    vTaskDelay(msec/portTICK_PERIOD_MS);
}

s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 bme280_status_error = BME280_INIT_VALUE;

    esp_err_t espRc;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, cnt, true);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
    if (espRc == ESP_OK) {
        bme280_status_error = SUCCESS;
    } else {
        bme280_status_error = FAIL;
    }
    i2c_cmd_link_delete(cmd);

    return (s8)bme280_status_error;
}

s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 bme280_status_error = BME280_INIT_VALUE;
    esp_err_t espRc;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    if (cnt > 1) {
        i2c_master_read(cmd, reg_data, cnt-1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data+cnt-1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
    if (espRc == ESP_OK) {
        bme280_status_error = SUCCESS;
    } else {
        bme280_status_error = FAIL;
    }

    i2c_cmd_link_delete(cmd);

    return (s8)bme280_status_error;
}

/*void bme280_read_normal(s32 com_rslt, bme280_measure* measure_buffer) {
    s32 v_uncomp_pressure_s32;
    s32 v_uncomp_temperature_s32;
    s32 v_uncomp_humidity_s32;
    
    if (com_rslt == SUCCESS) {
        
        ESP_LOGI(TAG,"Setup was successfull");

            com_rslt = bme280_read_uncomp_pressure_temperature_humidity(&v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32); //Actual I2C read

            if (com_rslt == SUCCESS) {
                measure_buffer->temperature = bme280_compensate_temperature_double(v_uncomp_temperature_s32);
                measure_buffer->pressure = bme280_compensate_pressure_double(v_uncomp_pressure_s32)/100;
                measure_buffer->humidity = bme280_compensate_humidity_double(v_uncomp_humidity_s32);
                ESP_LOGI(TAG, "Measure: %.2f degC / %.3f hPa / %.3f %%", measure_buffer->temperature, measure_buffer->pressure, measure_buffer->humidity);

            } else {
                ESP_LOGE(TAG, "Measure error. Code: %d", com_rslt);
            }

    } else {
        ESP_LOGE(TAG, "Initalization or setting error. Code: %d", com_rslt);
    }
}*/

/*
*   Public API
*/

void bme280_i2c_init() {

    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = GPIO_NUM_22,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };

    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    
}

s32 bme280_setup(u8 t_os, u8 p_os, u8 h_os, u8 filter_k) {  //TODO: com_rslt as static global variable
    struct bme280_t bme280 = {
        .bus_write = BME280_I2C_bus_write,
        .bus_read = BME280_I2C_bus_read,
        .dev_addr = BME280_I2C_ADDRESS,
        .delay_msec = BME280_delay_msec
    };

    com_rslt = bme280_init(&bme280);

    com_rslt += bme280_set_oversamp_temperature(t_os);
    com_rslt += bme280_set_oversamp_pressure(p_os);
    com_rslt += bme280_set_oversamp_humidity(h_os);

    com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_10_MS);
    com_rslt += bme280_set_filter(filter_k);

    com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);
    return com_rslt;
}

void bme280_read_normal() {
    bme280_measure measure_buffer;

    s32 com_rslt;
    s32 v_uncomp_pressure_s32;
    s32 v_uncomp_temperature_s32;
    s32 v_uncomp_humidity_s32;

    com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
        &v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

    if (com_rslt == SUCCESS) {
        measure_buffer.temperature = bme280_compensate_temperature_double(v_uncomp_temperature_s32);
        measure_buffer.pressure = bme280_compensate_pressure_double(v_uncomp_pressure_s32)/100;
        measure_buffer.humidity = bme280_compensate_humidity_double(v_uncomp_humidity_s32);
        ESP_LOGI(TAG, "Measure: %.2f degC / %.3f hPa / %.3f %%",
        measure_buffer.temperature, measure_buffer.pressure, measure_buffer.humidity);
    } else {
        ESP_LOGE(TAG, "Measure error. Code: %d", com_rslt);
    }
    
    vTaskDelay(1000/portTICK_PERIOD_MS);    //wait 1 second

}


void task_bme280_normal_mode()
{
    bme280_measure measure_buffer;
    s32 v_uncomp_pressure_s32, v_uncomp_temperature_s32, v_uncomp_humidity_s32;

    if (com_rslt == SUCCESS) {
        while(true) {

            com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
                &v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

            if (com_rslt == SUCCESS) {
                measure_buffer.temperature = bme280_compensate_temperature_double(v_uncomp_temperature_s32);
                measure_buffer.pressure = bme280_compensate_pressure_double(v_uncomp_pressure_s32)/100;
                measure_buffer.humidity = bme280_compensate_humidity_double(v_uncomp_humidity_s32);
                xSemaphoreTake(measure_sem, 1000/portTICK_PERIOD_MS);
                last_measure = measure_buffer;
                xSemaphoreGive(measure_sem);
                ESP_LOGD(TAG, "Measure: %.2f degC / %.3f hPa / %.3f %%",
                    measure_buffer.temperature, measure_buffer.pressure, measure_buffer.humidity);
            } else {
                ESP_LOGE(TAG, "Measure error. Code: %d", com_rslt);
            }

            vTaskDelay(1000/portTICK_PERIOD_MS);
        }
    } else {
        ESP_LOGE(TAG, "Initalization or setting error. Code: %d", com_rslt);
    }

    vTaskDelete(NULL);
}

bme280_measure bme280_get_last_measure() {
    xSemaphoreTake(measure_sem, 1000/portTICK_PERIOD_MS);
    bme280_measure buffer = measure_sem;
    xSemaphoreGive(measure_sem);
    return buffer;
}

/*
void task_bme280_forced_mode(void *ignore) {
    bme280_measure measure_buffer;
    struct bme280_t bme280 = {
        .bus_write = BME280_I2C_bus_write,
        .bus_read = BME280_I2C_bus_read,
        .dev_addr = BME280_I2C_ADDRESS,
        .delay_msec = BME280_delay_msec
    };

    s32 com_rslt;
    s32 v_uncomp_pressure_s32;
    s32 v_uncomp_temperature_s32;
    s32 v_uncomp_humidity_s32;

    com_rslt = bme280_init(&bme280);

    com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_1X);
    com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_1X);
    com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

    com_rslt += bme280_set_filter(BME280_FILTER_COEFF_OFF);
    if (com_rslt == SUCCESS) {
        while(true) {
            com_rslt = bme280_get_forced_uncomp_pressure_temperature_humidity(
                &v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

                if (com_rslt == SUCCESS) {
                    measure_buffer.temperature = bme280_compensate_temperature_double(v_uncomp_temperature_s32);
                    measure_buffer.pressure = bme280_compensate_pressure_double(v_uncomp_pressure_s32)/100;
                    measure_buffer.humidity = bme280_compensate_humidity_double(v_uncomp_humidity_s32);
                    ESP_LOGI(TAG, "Measure: %.2f degC / %.3f hPa / %.3f %%",
                        measure_buffer.temperature, measure_buffer.pressure, measure_buffer.humidity);
                    //mqtt_publish_measure(&measure_buffer);
                } else {
                    ESP_LOGE(TAG, "Measure error. Code: %d", com_rslt);
                }
            }
        } else {
            ESP_LOGE(TAG, "Initalization or setting error. Code: %d", com_rslt);
        }

    vTaskDelete(NULL);
}*/
