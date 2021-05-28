#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "stdint.h"
#include "stdbool.h"
#include "driver/gpio.h"
#include "adxl345.h"
#include "i2c/i2c.h"

static const char *TAG = "ADXl345";

// Coefficients to adjust acceleration direction.
// Change these values based on ADXL345's orientation.
static int adxl345_coefficients[] = {1, 1, -1};  // x, y, z

static uint8_t get_device_id() {
	uint8_t val;
	i2c_readbyte(ADXL345_I2C_ADDR, ADXL345_REG_DEVID, &val);
	return val;
}

/**
 * @return Exit code
 * @retval ESP_OK if ok
 * @retval != ESP_OK if some error occurred
 */
int adxl345_init(struct adxl345_i2c_conf *config) {
	struct adxl345_i2c_conf __conf = *config;

	i2c_init(__conf.sda, __conf.scl, __conf.bus);

	if (get_device_id() != ADXL345_DEVID)
		return ESP_FAIL;

	// Turning on ADXL345
	ESP_LOGD(TAG, "Turning ADXL345 on, remember to call i2c_init before this.");
	
	// Enable measurements
	i2c_writebyte(ADXL345_I2C_ADDR, ADXL345_REG_POWER_CTL, 0x08);
	
	// Set data range
	uint8_t format;
	i2c_readbyte(ADXL345_I2C_ADDR, ADXL345_REG_DATA_FORMAT, &format);
	format &= ~0x0f;
	format |= __conf.range;
	format |= 0x08;
	i2c_writebyte(ADXL345_I2C_ADDR, ADXL345_REG_DATA_FORMAT, format);

	// Set datarate
	i2c_writebyte(ADXL345_I2C_ADDR, ADXL345_REG_BW_RATE, 0x09);
	
	return ESP_OK;
}

void adxl345_get_data(struct accelerometer_data *result) {
	uint8_t buff[A_TO_READ];
	int16_t x, y, z;
	i2c_read(ADXL345_I2C_ADDR, ADXL345_REG_DATAX0, buff, A_TO_READ);
	x = (buff[1] << 8) | buff[0];
	y = (buff[3] << 8) | buff[2];
	z = (buff[5] << 8) | buff[4];
	result->x = x * ADXL345_MG2G_MULTIPLIER * GRAVITY_EARTH * adxl345_coefficients[0];
	result->y = y * ADXL345_MG2G_MULTIPLIER * GRAVITY_EARTH * adxl345_coefficients[1];
	result->z = z * ADXL345_MG2G_MULTIPLIER * GRAVITY_EARTH * adxl345_coefficients[2];
}