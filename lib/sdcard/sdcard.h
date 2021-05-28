/**
 * @file sdcard.h
 * @author sCanSati Team 2020-2021, ITIS E. Fermi, Modena
 * @date 12 May 2021
 * @brief Tiny library for sd card (SDSPI) initialization and management
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

/**
 * @brief initialize spi bus, sd card and vfs
 * @param conf configuration structure
 * @return ESP_OK if no error occurred; the error value otherwise
 */
esp_err_t sdcard_init(const struct sdcard_config *conf);

/**
 * @brief open a file after a few settings
 * @param filename string representing the relative name of the file you want to open
 * @return stream linked to the open file
 */
FILE *sdcard_get_stream(const char *filename);

#ifdef __cplusplus
}
#endif

#endif  // __sdcard_h