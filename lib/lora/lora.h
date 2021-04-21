/**
 * @file lora.h
 * @author CanSat team
 * @date 21 Apr 2021
 * @brief LoRa wrapper for RFM95 (RFM9_) module
 */

#include "rfm95.h"

typedef struct {
    int timestamp;
    EventBits_t contains;
    bme280_data_t ambient;
    gps_position_t position;
    struct sps30_measurement partmatter;
} Payload_t;
