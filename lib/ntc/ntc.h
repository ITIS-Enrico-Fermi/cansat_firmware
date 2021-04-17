#ifndef __ntc_h
#define __ntc_h

struct ntc_config {
    
}

int ntc_config(struct ntc_config *config);
void ntc_task(void *pvParameters);

#endif