//
//  gps.h
//  
//
//  Created by Giulio Corradini on 26/01/2020.
//

#ifndef gps_h
#define gps_h

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "minmea.h"

void gps_configure(uart_port_t uart_controller_port);
char* gps_uart_read_line();
void gps_location_update_handler(void *pvParameters);

#define GPS_FIXED_POSITION      0x01
#define GPS_LOCATION_UPDATED    0x02
void gps_set_location_task_handle(TaskHandle_t task_handle);

typedef struct minmea_sentence_gga gps_position_t;
_Bool gps_get_current_position(gps_position_t* position_buffer);

EventGroupHandle_t* gps_get_status();

#endif /* gps_h */
