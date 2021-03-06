#ifndef __SPS30_QUERY_H
#define __SPS30_QUERY_H

#include "sps30.h"

struct sps30_task_parameters {
    void    *dev_barrier;
    void    *pm_queue;
};

void sps30_task(void *pvParameters);

#endif  //__SPS30_QUERY_H