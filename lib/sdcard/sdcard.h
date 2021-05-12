/*
 * sdcard.h
 * 
 * SD Card over SPI bus facility
 * by Giulio Corradini
 * 
 * Cansat Firmware 2021
 * 
 */

#ifndef __sdcard_h
#define __sdcard_h

#include <stdbool.h>
#include <esp_err.h>

#define MOUNT_POINT "/sdcard"

struct spi {
    int miso;
    int mosi;
    int sck;
    int cs;
};

struct sdcard_config {
    struct spi spi;
    int freq;
    bool format_sd_if_mount_failed;
    int max_files;
};

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t sdcard_init(const struct sdcard_config *conf);
FILE *sdcard_get_fd(const char *filename);

#ifdef __cplusplus
}
#endif

#endif  // __sdcard_h