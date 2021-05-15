//
//  gps.c
//  
//
//  Created by Giulio Corradini on 26/01/2020.
//

#include "gps.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "minmea.h"
#include "linkedlist.h"

#define RX_BUF_SIZE 1024
#define TAG "GPS"


typedef struct {
    GPSConfig_t config;
    SemaphoreHandle_t lock;
    gps_position_t current_position;
    TaskHandle_t service_worker;
} __gpsdevice;

static Node* managed_devices = NULL;

GPSDevice_t gps_setup_new(GPSConfig_t *config) {
    __gpsdevice *new_device = malloc(sizeof(__gpsdevice));

    memcpy(&new_device->config, config, sizeof(GPSConfig_t));

    //UART settings
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
    };
    // Configure UART parameters
    printf("%d\n", config->uart_controller_port);
    ESP_ERROR_CHECK(uart_param_config(config->uart_controller_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(config->uart_controller_port, config->uart.tx, config->uart.tx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));  //IMPORTANT: specify pins
    
    const int uart_buffer_size = (1024 * 2);
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(config->uart_controller_port, uart_buffer_size, 0, 0, NULL, 0));

    new_device->lock = xSemaphoreCreateBinary();
    xSemaphoreGive(new_device->lock);
    
    if(managed_devices == NULL) {
        managed_devices = createNode(new_device, NULL, NULL);   //Create head
    } else {
        llAdd(managed_devices, new_device);
    }

    return (GPSDevice_t)new_device;
}


char* gps_uart_read_line(__gpsdevice *dev) {
    static char line[MINMEA_MAX_LENGTH];
    char *ptr = line;
    while(1) {
    
        int num_read = uart_read_bytes(dev->config.uart_controller_port, (unsigned char *)ptr, 1, portMAX_DELAY);
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
    __gpsdevice* dev = (__gpsdevice*)pvParameters;
    dev->service_worker = xTaskGetCurrentTaskHandle();

    enum minmea_gsa_fix_type gps_fix_type = MINMEA_GPGSA_FIX_NONE;
    
    while(true) {
        char *line = gps_uart_read_line(dev);
        
        if(!minmea_check(line, false)) {
            ESP_LOGE("GPS", "Received line is not a valid sentence. %s", line);
        } else {
        
            switch(minmea_sentence_id(line, false)) {
                case MINMEA_SENTENCE_GGA: {
                    struct minmea_sentence_gga frame;
                    minmea_parse_gga(&frame, line);
                    if(gps_fix_type > MINMEA_GPGSA_FIX_2D) {
                        ESP_LOGD("GPS", "GGA - Lat: %.6f. Lon: %.6f. Alt: %.6f %c.", minmea_tocoord(&frame.latitude), minmea_tocoord(&frame.longitude), minmea_tocoord(&frame.altitude), frame.altitude_units);
                        xEventGroupSetBits(dev->config.gps_status, GPS_FIXED_POSITION);
                    } else {
                        ESP_LOGW("GPS", "Cannot show altitude. GPS fix type is not 3D");
                        xEventGroupClearBits(dev->config.gps_status, GPS_LOCATION_UPDATED);
                    }
                    
                    if( xSemaphoreTake(dev->lock, (TickType_t) 10) == pdTRUE ) {
                        dev->current_position = frame;
                        xSemaphoreGive(dev->lock);
                        xEventGroupSetBits(dev->config.sync_barrier, dev->config.sync_id);
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

bool gps_get_current_position(GPSDevice_t d, gps_position_t* dst) {
    __gpsdevice *dev = (__gpsdevice*)d;

    if(dev->lock == NULL) {
        ESP_LOGW(TAG, "Position semaphore is null, you ran out of heap memory or you didn't initialize the GPS module.");
        return false;
    }
    
    if( xSemaphoreTake(dev->lock, (TickType_t) 10) == pdTRUE ) {
        *dst = dev->current_position;
        xSemaphoreGive(dev->lock);
        return true;
    } else {
        ESP_LOGE(TAG, "Couldn't take semaphore lock within 10 ticks. Secure access to current position can't be made");
        return false;
    }
}

void gps_free(GPSDevice_t d) {
    __gpsdevice* dev = (__gpsdevice*)d;

    vTaskDelete(dev->service_worker);

    uart_driver_delete(dev->config.uart_controller_port);

    vSemaphoreDelete(dev->lock);

    free(dev);
}