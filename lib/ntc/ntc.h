#ifndef __ntc_h
#define __ntc_h

struct ntc_config {
    void *sync_barrier;
    void *ntc_queue;
    int device_id;
};

int ntc_init(struct ntc_config *config);
void ntc_task(void *pvParameters);

#endif