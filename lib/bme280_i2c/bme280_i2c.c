#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/i2c.h>
#include "bme280.h"
#include "bme280_i2c.h"

#define TAG  "BME280 I2C"

static int8_t rslt;
static struct bme280_dev dev;
static uint32_t req_delay;

static uint8_t i2c_num_glob;

static SemaphoreHandle_t measure_sem;
static EventGroupHandle_t bme280_status;
static EventGroupHandle_t sync_barrier;
static int sync_id;

static bme280_data_t last_measure;

static TaskHandle_t handler;

//#pragma region hardware_access_interface

/**
 * @brief Delays execution of task by spec. milliseconds
 * Uses FreeRTOS API to put the task in blocked mode for the amount of milliseconds specified
 * @param msec milliseconds to wait
 */
void BME280_delay_msec(uint32_t msec)
{
    vTaskDelay(msec/portTICK_PERIOD_MS);
}

/**
 * @brief I2C bus write function for BME280
 * @param dev_addr The address of BME280
 * @param reg_addr Register to write on
 * @param reg_data Pointer to contiguos data block to write on @reg_addr register
 * @param cnt @reg_data content lenght
 * @return int8_t with sensor status
 */
int8_t BME280_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t cnt)
{
    int8_t bme280_status_error = 0;

    esp_err_t espRc;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, cnt, true);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(i2c_num_glob, cmd, 10/portTICK_PERIOD_MS);
    if (espRc == ESP_OK) {
        bme280_status_error = 0;
    } else {
        bme280_status_error = 1;
    }
    i2c_cmd_link_delete(cmd);

    return bme280_status_error;
}

/**
 * @brief I2C bus write function for BME280
 * @param dev_addr The address of BME280
 * @param reg_addr Register to read from
 * @param reg_data Pointer to buffer to write @reg_addr register content on
 * @param cnt @reg_data buffer size
 * @return int8_t with sensor status
 */
int8_t BME280_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t cnt)
{
    int32_t bme280_status_error = 0;
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

    espRc = i2c_master_cmd_begin(i2c_num_glob, cmd, 10/portTICK_PERIOD_MS);
    if (espRc == ESP_OK) {
        bme280_status_error = 0;
    } else {
        bme280_status_error = 1;
    }

    i2c_cmd_link_delete(cmd);

    return bme280_status_error;
}

//#pragma endregion

//#pragma region api
/*
*   Public API
*/

void bme280_i2c_init(uint8_t i2c_sda, uint8_t i2c_scl, uint8_t i2c_num) {

    ESP_LOGD(
        TAG,
        "i2c_sda: %d\n"
        "i2c_scl: %d\n"
        "i2c_num: %d\n",
        i2c_sda,
        i2c_scl,
        i2c_num
    );

    i2c_num_glob = i2c_num;

    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = i2c_scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };

    ESP_ERROR_CHECK(i2c_param_config(i2c_num, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_num, I2C_MODE_MASTER, 0, 0, 0));
    
}

/**
 * @brief BME280 setup
 * 
 * @param t_os Temperature oversampling rate [BME280_OVERSAMPLING_nX]
 * @param p_os Pressure oversampling rate [BME280_OVERSAMPLING_nX]
 * @param h_os Humidity oversampling rate [BME280_OVERSAMPLING_nX]
 * @param filter_k Low-pass filter coefficient [BME280_FILTER_COEFF_n/OFF]
 * 
 */
void bme280_setup(bme280_config_t *config) {  //Temperature Oversampling, Pressure Oversampling, Humidity Oversampling, Filter Coefficient
    
    uint8_t t_os = config->t_os;
    uint8_t p_os = config->p_os;
    uint8_t h_os = config->h_os;
    uint8_t filter_k = config->filter_k;
    uint8_t i2c_sda = config->i2c.sda;
    uint8_t i2c_scl = config->i2c.scl;
    uint8_t i2c_num = config->i2c.num;

    handler = config->parent_task;

    dev = (struct bme280_dev){
        .dev_id = BME280_I2C_ADDR_PRIM,
        .intf = BME280_I2C_INTF,
        .read = BME280_I2C_bus_read,
        .write = BME280_I2C_bus_write,
        .delay_ms = BME280_delay_msec,
        .settings = {
            .osr_t = t_os,
            .osr_p = p_os,
            .osr_h = h_os,
            .filter = filter_k
        }
    };
    uint8_t settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    bme280_i2c_init(i2c_sda, i2c_scl, i2c_num);

    rslt = BME280_OK;
    rslt = bme280_init(&dev);
    if (rslt != BME280_OK)
        ESP_LOGE(TAG,
            "rslt"
            "\tAfter init: %d\n", rslt);
    rslt = bme280_set_sensor_settings(settings_sel, &dev);
    if (rslt != BME280_OK)
        ESP_LOGE(TAG, "\tAfter set_sensor_settings: %d\n", rslt);
    rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);
    if (rslt != BME280_OK)
        ESP_LOGE(TAG, "\tAfter set_sensor_mode: %d\n", rslt);

    req_delay = bme280_cal_meas_delay(&dev.settings);


    measure_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(measure_sem);

    sync_barrier = config->sync_barrier;
    sync_id = config->sync_id;

    bme280_status = xEventGroupCreate();

    bme280_set_delay(config->delay);

}

/**
 * @brief Measure FreeRTOS task
 * FreeRTOS task to enable measures cycle and 
 */
void bme280_task_normal_mode(void* pv_params)
{
    struct bme280_data comp_data;

    if (rslt == BME280_OK) {

        while(true) {

            dev.delay_ms(req_delay);
            
            rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
            xSemaphoreTake(measure_sem, req_delay/portTICK_PERIOD_MS);
            last_measure = comp_data;
            xSemaphoreGive(measure_sem);
            xEventGroupSetBits(sync_barrier, sync_id);
            ESP_LOGD(TAG, "T: %.2f, P: %.2f, h: %.2f", comp_data.temperature, comp_data.pressure, comp_data.humidity);

        }
    } else {
        ESP_LOGE(TAG, "Initalization or setting error. Code: %d", rslt);
    }

    vTaskDelete(NULL);
}

bme280_data_t bme280_get_last_measure() {
    xSemaphoreTake(measure_sem, req_delay/portTICK_PERIOD_MS);
    bme280_data_t buffer = last_measure;
    xSemaphoreGive(measure_sem);
    return buffer;
}

EventGroupHandle_t* bme280_get_status() {
    return &bme280_status;
}

void bme280_register_handler(TaskHandle_t _handler) {
    handler = _handler;
}

void bme280_set_delay(uint32_t delay_ms) {
    req_delay = delay_ms;
}
//#pragma endregion

/*
    SDA     21
    SCL     22
*/