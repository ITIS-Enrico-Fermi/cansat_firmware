#include "sdcard.h"
#include "driver/sdspi_host.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_types.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include <stdio.h>
#include <string.h>

// #define ESP_LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <esp_log.h>

#define TAG "SDCARD SPI"

#define SPI_NO_PIN          (-1)

#define BLOCK_SIZE          (16)  // Kb
#define MAX_TRANSFER_SIZE   (64)  // Default value is 4096. Max is 64 when DMA_CHAN == 0
#define MAX_FILES           (1)

#define DMA_CHAN            (host.slot)

static FILE* sdcard_fd;

esp_err_t sdcard_init(const struct sdcard_config *conf) {
    sdmmc_card_t *card;
    const char mp[] = MOUNT_POINT;
    esp_err_t ret;

    ESP_LOGD(
        TAG,
        "Using parameters\n"
        "\tmiso: %d\n"
        "\tmosi: %d\n"
        "\tsck: %d\n"
        "\tcs: %d",
        conf->spi.miso,
        conf->spi.mosi,
        conf->spi.sck,
        conf->spi.cs
    );

    gpio_set_pull_mode(conf->spi.miso, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(conf->spi.mosi, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(conf->spi.sck, GPIO_PULLUP_ONLY);

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = HSPI_HOST;
    spi_bus_config_t bus_conf = {
        .miso_io_num = conf->spi.miso,
        .mosi_io_num = conf->spi.mosi,
        .sclk_io_num = conf->spi.sck,
        .quadhd_io_num = SPI_NO_PIN,
        .quadwp_io_num = SPI_NO_PIN,
        .max_transfer_sz = MAX_TRANSFER_SIZE
    };
    ret = spi_bus_initialize(host.slot, &bus_conf, DMA_CHAN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed spi bus init: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGD(TAG, "Spi bus initialized");

    sdspi_device_config_t slot_conf = {
        .gpio_cs = conf->spi.cs,
        .gpio_cd = SDSPI_SLOT_NO_CD,
        .gpio_wp = SDSPI_SLOT_NO_WP,
        .gpio_int = SDSPI_SLOT_NO_INT,
        .host_id = host.slot
    };

    sdspi_dev_handle_t dev_handler;

    ret = sdspi_host_init_device(&slot_conf, &dev_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init host device: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGD(TAG, "Initialized host device");

    esp_vfs_fat_sdmmc_mount_config_t mount_conf = {
        .format_if_mount_failed = conf->format_sd_if_mount_failed,
        .max_files = conf->max_files == NULL ? MAX_FILES : conf->max_files,
        .allocation_unit_size = BLOCK_SIZE * 1024
    };
    ret = esp_vfs_fat_sdspi_mount(mp, &host, &slot_conf, &mount_conf, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init and mount the card: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGD(TAG, "SD vfs mounted");

    char info_buf[128] = "\0";
    FILE *info_buf_stream;

    info_buf_stream = fmemopen(info_buf, sizeof(info_buf)*sizeof(char), "w");

    fprintf(info_buf_stream, "SD CARD INFO\n");
    sdmmc_card_print_info(info_buf_stream, card);

    fflush(info_buf_stream);
    fclose(info_buf_stream);
    
    ESP_LOGI(TAG, "%s", info_buf);

    return ESP_OK;
}