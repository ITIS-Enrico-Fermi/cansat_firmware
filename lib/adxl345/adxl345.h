#ifndef __ADXL345_H__
#define __ADXL345_H__

#define A_TO_READ (6)
#define ADXL345_DEVID  (0xe5)

// Address
#define ADXL345_I2C_ADDR (0x53)//(0xA7>>1)

// Registers
#define ADXL345_REG_DEVID (0x00)        // Device ID
#define ADXL345_REG_THRESH_TAP (0x1d)   // Tap threshold
#define ADXL345_REG_OFSX (0x1e)         // X-axis offset
#define ADXL345_REG_OFSY (0x1f)         // Y-axis offset
#define ADXL345_REG_OFSZ (0x20)         // Z-axis offset
#define ADXL345_REG_DUR (0x21)          // Tap duration
#define ADXL345_REG_LATENT (0x22)       // Tap latency
#define ADXL345_REG_WINDOW (0x23)       // Tap window
#define ADXL345_REG_THRESH_ACT (0x24)   // Activity threshold
#define ADXL345_REG_THRESH_INACT (0x25) // Inactivity threshold
#define ADXL345_REG_TIME_INACT (0x26)   // Inactivity time
#define ADXL345_REG_ACT_INACT_CTL                                              \
    (0x27) // Axis enable control for activity and inactivity detection
#define ADXL345_REG_THRESH_FF (0x28) // Free-fall threshold
#define ADXL345_REG_TIME_FF (0x29)   // Free-fall time
#define ADXL345_REG_TAP_AXES (0x2a)  // Axis control for single/double tap
#define ADXL345_REG_ACT_TAP_STATUS (0x2b) // Source for single/double tap
#define ADXL345_REG_BW_RATE (0x2c)        // Data rate and power mode control
#define ADXL345_REG_POWER_CTL (0x2d)      // Power-saving features control
#define ADXL345_REG_INT_ENABLE (0x2e)     // Interrupt enable control
#define ADXL345_REG_INT_MAP (0x2f)        // Interrupt mapping control
#define ADXL345_REG_INT_SOURCE (0x30)     // Source of interrupts
#define ADXL345_REG_DATA_FORMAT (0x31)    // Data format control
#define ADXL345_REG_DATAX0 (0x32)         // X-axis data 0
#define ADXL345_REG_DATAX1 (0x33)         // X-axis data 1
#define ADXL345_REG_DATAY0 (0x34)         // Y-axis data 0
#define ADXL345_REG_DATAY1 (0x35)         // Y-axis data 1
#define ADXL345_REG_DATAZ0 (0x36)         // Z-axis data 0
#define ADXL345_REG_DATAZ1 (0x37)         // Z-axis data 1
#define ADXL345_REG_FIFO_CTL (0x38)       // FIFO control
#define ADXL345_REG_FIFO_STATUS (0x39)    // FIFO status

#define ADXL345_MG2G_MULTIPLIER (0.004)  // < 4mg per lsb

#define GRAVITY_EARTH (9.80665F)  // Earth's gravity in m/s^2

typedef enum {
  ADXL345_RANGE_16_G = 0b11, // +/- 16g
  ADXL345_RANGE_8_G = 0b10,  // +/- 8g
  ADXL345_RANGE_4_G = 0b01,  // +/- 4g
  ADXL345_RANGE_2_G = 0b00   // +/- 2g (default value)
} range_t;


struct accelerometer_data {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct adxl345_i2c_conf {
    int sda;
    int scl;
    int bus;
    range_t range;

    void *data_queue;  // QueueHandle_t
    void *sync_barrier;  // EventGroupHandle_t
    int device_id;
};

int adxl345_init(struct adxl345_i2c_conf *config);
void adxl345_get_data(struct accelerometer_data *result);

void accelerometer_task(void *param);

#endif