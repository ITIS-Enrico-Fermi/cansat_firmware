#ifndef __ADXL345_H__
#define __ADXL345_H__

struct accelerometer_data {
    int x;
    int y;
    int z;
};

void adxl345_init();
void adxl345_get_data(struct accelerometer_data *result);

#endif