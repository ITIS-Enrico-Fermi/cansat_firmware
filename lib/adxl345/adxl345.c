#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "stdint.h"
#include "stdbool.h"
#include "driver/gpio.h"
#include "adxl345.h"
#include "i2c/i2c.h"

#define ADXL345_I2C_ADDR (0x53)//(0xA7>>1)
#define A_TO_READ (6)
static const char *TAG = "adxl345";

void adxl345_init() {
	// Turning on ADXL345
	ESP_LOGD(TAG, "Turning ADXL345 on, remember to call i2c_init before this.");
	i2c_writebyte(ADXL345_I2C_ADDR, 0x2D, 1 << 3);
	i2c_writebyte(ADXL345_I2C_ADDR, 0x31, 0x0B);
	i2c_writebyte(ADXL345_I2C_ADDR, 0x2C, 0x09);
}

void adxl345_get_data(struct accelerometer_data *result) {
	uint8_t regAddress = 0x32;
	uint8_t buff[A_TO_READ];
	i2c_read(ADXL345_I2C_ADDR, regAddress, buff, A_TO_READ);
	result->x = (((int) buff[1]) << 8) | buff[0];
	result->y = (((int) buff[3]) << 8) | buff[2];
	result->z = (((int) buff[5]) << 8) | buff[4];
}