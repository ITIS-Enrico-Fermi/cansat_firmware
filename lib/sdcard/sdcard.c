#include "sdcard.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "driver/sdmmc_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

void sdcard_init() {

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = VSPI_HOST;

    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    slot_config.gpio_miso = 34;
    slot_config.gpio_mosi = 33;
    slot_config.gpio_sck = 32;
    slot_config.gpio_cs = 27;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5
    };

    sdmmc_card_t* card;

    ESP_ERROR_CHECK(esp_vfs_fat_sdmmc_mount(SD_MOUNTPOINT, &host, &slot_config, &mount_config, &card));

    sdmmc_card_print_info(stdout, card);

}