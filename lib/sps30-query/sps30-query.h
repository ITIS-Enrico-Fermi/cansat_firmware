#ifndef __SPS30_QUERY_H
#define __SPS30_QUERY_H

#include "sps30.h"

struct sps30_task_parameters {
    void    *dev_barrier;
    void    *pm_queue;
    int     device_id;
};

#define SPS30_I2C_ALONE

void sps30_task(void *pvParameters);

#endif  //__SPS30_QUERY_H