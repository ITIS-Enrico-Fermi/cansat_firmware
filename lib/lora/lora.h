/**
 * @file lora.h
 * @author CanSat team
 * @date 21 Apr 2021
 * @brief LoRa wrapper for RFM95 (RFM9_) module
 */

#ifndef __LORA
#define __LORA

#include <time.h>
#include <stdint.h>
#include <stdbool.h>
#include <freertos/event_groups.h>

#include "rfm95.h"

#include "bme280_i2c.h"
#include "gps.h"
#include "sps30.h"

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

typedef struct {
    int timestamp;
    EventBits_t contains;
    bme280_data_t ambient;
    gps_position_t position;
    struct sps30_measurement partmatter;
} Payload_t;


#pragma pack(1)
struct lora_bme280_data {
    double p;
    double t;
    double h;
};

#pragma pack(1)
struct lora_gps_data {
    int64_t lat;
    int64_t lon;
    int64_t alt;
};

#pragma pack(1)
struct lora_sps30_data {
    float mc_1p0;
    float mc_2p5;
    float mc_4p0;
    float mc_10p0;
    float nc_0p5;
    float nc_1p0;
    float nc_2p5;
    float nc_4p0;
    float nc_10p0;
    float typical_particle_size;
};


#pragma pack(1)  // Avoid struct padding
struct lora_shrinked_payload {  // Shared struct between base station and transmitter
    time_t timestamp;
    struct lora_bme280_data bme280;
    struct lora_gps_data gps;
    struct lora_sps30_data sps30;
};

struct lora_cfg {
    struct rfm95_spi_config spi;
    long freq;
    int tp;  // Transmission Power
    int sf;  // Spreading factor
    int bw;  // Bandwidth
    bool is_crc_en;  // Is CRC8 enabled?
};

// Functions
void lora_setup(struct lora_cfg *cfg);
// void lora_shrink_payload(Payload_t *src, struct shrinked_payload *dst);
// void lora_set_payload(struct shrinked_payload *sp);
// void lora_shrink_and_set_payload(Payload_t *p);
// void lora_send_payload(struct shrinked_payload *sp);
void lora_send(Payload_t *p);

// // Tasks
void lora_transmission_task(void *pv);

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif // __LORA