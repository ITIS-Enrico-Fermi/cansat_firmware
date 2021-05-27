/*
 *  i2c.c
 * 
 *  by Giulio Corradini
 *  Cansat Firmware 2021
 * 
 */

#include "i2c.h"
#include "driver/i2c.h"

static i2c_port_t controller = I2C_NUM_1;
SemaphoreHandle_t i2c_mutex = NULL;

void i2c_init(int sda, int scl, int bus) {
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    controller = bus;

    if(!i2c_mutex) {
        i2c_mutex = xSemaphoreCreateMutex();
        xSemaphoreGive(i2c_mutex);
    }
}

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int i2c_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length) {
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1 | I2C_MASTER_WRITE), true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, length, true);
    i2c_master_stop(cmd);

    xSemaphoreTake(i2c_mutex, 1000 / portTICK_RATE_MS);
    err = i2c_master_cmd_begin(controller, cmd, 1000 / portTICK_RATE_MS);
    xSemaphoreGive(i2c_mutex);

    return err;
}

int i2c_writebyte(uint8_t i2c_addr, uint8_t reg_addr, uint8_t reg_data) {
    return i2c_write(i2c_addr, reg_addr, &reg_data, 1);
}

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : Sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int i2c_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length) {
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    //  Write pointer to register (it'll be read in the next transaction)
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1 | I2C_MASTER_WRITE), true);
    i2c_master_write_byte(cmd, reg_addr, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1 | I2C_MASTER_READ), true);
    i2c_master_read(cmd, reg_data, length, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

        
    xSemaphoreTake(i2c_mutex, 1000 / portTICK_RATE_MS);
    err = i2c_master_cmd_begin(controller, cmd, 1000 / portTICK_RATE_MS);
    xSemaphoreGive(i2c_mutex);

    return err;
}

int i2c_readbyte(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data) {
    return i2c_read(i2c_addr, reg_addr, reg_data, 1);
}