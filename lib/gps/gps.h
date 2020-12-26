//
//  gps.h
//  
//
//  Created by Giulio Corradini on 26/01/2020.
//

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "minmea.h"

#ifndef gps_h
#define gps_h

#ifdef __cplusplus
extern "C" {
#endif

#define GPS_FIXED_POSITION      0x01
#define GPS_LOCATION_UPDATED    0x01

typedef struct {
    uart_port_t uart_controller_port;
    TaskHandle_t parent_task;
    EventGroupHandle_t sync_barrier;
    EventGroupHandle_t gps_status;
} GPSConfig_t;

void gps_setup(GPSConfig_t *config);
void gps_task(void *pvParameters);

typedef struct minmea_sentence_gga gps_position_t;
_Bool gps_get_current_position(gps_position_t* position_buffer);


#ifdef __cplusplus
}
#endif
#endif /* gps_h */

/*
RX to pin 17 TX2
TX to pin 16 RX2
*/