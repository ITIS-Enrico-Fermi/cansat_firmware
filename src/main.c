#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG "Main"

#include "driver/uart.h"
#include "stdio.h"
#include "minmea.h" //GPS NMEA sentence analysis

#include "gps.h"

void app_main() {
    gps_configure(UART_NUM_2);
    
    xTaskCreate(gps_location_update_handler, "GPS_location_task", 2048, NULL, 10, NULL);
    
    EventGroupHandle_t* gps_status = gps_get_status();
    
    while(xEventGroupWaitBits(*gps_status, GPS_FIXED_POSITION, pdFALSE, pdFALSE, pdMS_TO_TICKS(2000)) == pdFALSE);
    
    while(1) {
        
        //Wait for notification update
        if( xTaskNotifyWait(GPS_LOCATION_UPDATED, 0x00, NULL, pdMS_TO_TICKS(1100)) == pdTRUE ) {
            gps_position_t current_position;
            if(gps_get_current_position(&current_position)) {
                printf("Latitude: %.6f, Longitude: %.6f, Altitude: %.6f, Fix quality: %d\n", minmea_tocoord(&current_position.latitude), minmea_tocoord(&current_position.longitude), minmea_tocoord(&current_position.altitude), current_position.fix_quality);
            }
        } else {
            ESP_LOGW("Main", "Location was not updated within 1 second");
        }
    }
}