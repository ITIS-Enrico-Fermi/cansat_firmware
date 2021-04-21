/**
 * @file lora.h
 * @author CanSat team
 * @date 21 Apr 2021
 * @brief LoRa wrapper for RFM95 (RFM9_) module
 */

#ifndef __LORA
#define __LORA

#include "rfm95.h"
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

#pragma pack(1)  // Avoid struct padding
struct lora_payload {  // Shared struct between base station and transmitter
    time_t timestamp;
    bme280_data_t ambient;
    gps_position_t position;
    struct sps30_measurement partmatter;
}

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif __LORA  // __LORA