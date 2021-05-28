//
//  gps.h
//  
//
//  Created by sCanSati Team 2020-2021, ITIS E. Fermi, Modena on 26/01/2020.
//

#include "stdbool.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "minmea.h"

#ifndef gps_h
#define gps_h

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GPS_FIXED_POSITION      =   (0x01 << 0),
    GPS_LOCATION_UPDATED    =   (0x01 << 1),
    GPS_STATUS_TOTAL
} GPSStatus_t;

struct uart_config {
    int tx;
    int rx;
};

typedef struct {
    uart_port_t uart_controller_port;
    TaskHandle_t parent_task;
    EventGroupHandle_t gps_status;
    EventGroupHandle_t sync_barrier;
    size_t sync_id;
    struct uart_config uart;
} GPSConfig_t;

typedef void* GPSDevice_t;

GPSDevice_t gps_setup_new(GPSConfig_t *config);
void gps_task(void *pvParameters);
void gps_free(GPSDevice_t dev);

typedef struct minmea_sentence_gga gps_position_t;
bool gps_get_current_position(GPSDevice_t dev, gps_position_t* dst);


#ifdef __cplusplus
}
#endif
#endif /* gps_h */

/*
RX to pin 17 TX2
TX to pin 16 RX2
*/