//
//  gps.c
//  
//
//  Created by Giulio Corradini on 26/01/2020.
//

#include "gps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "minmea.h"
#include "esp_log.h"
#include "string.h"

#define RX_BUF_SIZE 1024
#define TAG "GPS"

static gps_position_t current_position;
static SemaphoreHandle_t position_semaphore = NULL;
static TaskHandle_t location_task_handle;
static EventGroupHandle_t gps_status;

static GPSConfig_t config;

void gps_setup(GPSConfig_t *global_config) {
    memcpy(&config, global_config, sizeof(GPSConfig_t));

    //UART settings
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
    };
    // Configure UART parameters
    printf("%d\n", config.uart_controller_port);
    ESP_ERROR_CHECK(uart_param_config(config.uart_controller_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(config.uart_controller_port, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));  //IMPORTANT: specify pins
    
    const int uart_buffer_size = (1024 * 2);
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(config.uart_controller_port, uart_buffer_size, 0, 0, NULL, 0));
    
    gps_status = config.gps_status;
    location_task_handle = config.parent_task;

    position_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(position_semaphore);
    
}


char* gps_uart_read_line() {

    static char line[MINMEA_MAX_LENGTH];
    char *ptr = line;
    while(1) {
    
        int num_read = uart_read_bytes(config.uart_controller_port, (unsigned char *)ptr, 1, portMAX_DELAY);
        if(num_read == 1) {
        
            // new line found, terminate the string and return
            if(*ptr == '\n') {
                ptr++;
                *ptr = '\0';
                return line;
            }
            
            // else move to the next char
            ptr++;
        }
    }
}

void gps_task(void *pvParameters) {
    enum minmea_gsa_fix_type gps_fix_type = MINMEA_GPGSA_FIX_NONE;
    
    while(true) {
        char *line = gps_uart_read_line();
        
        if(!minmea_check(line, false)) {
            ESP_LOGE("GPS", "Received line is not a valid sentence. %s", line);
        } else {
        
            switch(minmea_sentence_id(line, false)) {
                case MINMEA_SENTENCE_GGA: {
                    struct minmea_sentence_gga frame;
                    minmea_parse_gga(&frame, line);
                    if(gps_fix_type > MINMEA_GPGSA_FIX_2D) {
                        ESP_LOGD("GPS", "GGA - Lat: %.6f. Lon: %.6f. Alt: %.6f %c.", minmea_tocoord(&frame.latitude), minmea_tocoord(&frame.longitude), minmea_tocoord(&frame.altitude), frame.altitude_units);
                        xEventGroupSetBits(gps_status, GPS_FIXED_POSITION);
                    } else {
                        ESP_LOGW("GPS", "Cannot show altitude. GPS fix type is not 3D");
                        xEventGroupClearBits(gps_status, GPS_LOCATION_UPDATED);
                    }
                    
                    if( xSemaphoreTake(position_semaphore, (TickType_t) 10) == pdTRUE ) {
                        current_position = frame;
                        xSemaphoreGive(position_semaphore);
                        xTaskNotify(location_task_handle, GPS_LOCATION_UPDATED, eSetBits);
                        ESP_LOGD(TAG, "Notified a location update");
                    } else {
                        ESP_LOGE(TAG, "Couldn't lock position semaphore within 10 ticks");
                    }
                    
                    break;
                }
                case MINMEA_SENTENCE_GSA: {
                    struct minmea_sentence_gsa frame;
                    minmea_parse_gsa(&frame, line);
                    gps_fix_type = frame.fix_type;
                    if(frame.fix_type == MINMEA_GPGSA_FIX_NONE)
                        ESP_LOGW("GPS", "GPS has no fix. Cannot determine position.");
                    break;
                }
                default:
                    //ESP_LOGD("GPS", "No action associated with this NMEA sentence. %s", line);
                    break;
            }
        
        }
        
    }
    
    vTaskDelete(NULL);
}

_Bool gps_get_current_position(gps_position_t* position_buffer) {
    if(position_semaphore == NULL) {
        ESP_LOGW(TAG, "Position semaphore is null, you ran out of heap memory or you didn't initialize the GPS module.");
        return false;
    }
    
    if( xSemaphoreTake(position_semaphore, (TickType_t) 10) == pdTRUE ) {
        *position_buffer = current_position;
        xSemaphoreGive(position_semaphore);
        return true;
    } else {
        ESP_LOGE(TAG, "Couldn't take semaphore lock within 10 ticks. Secure access to current position can't be made");
        return false;
    }
}
