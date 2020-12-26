#include "spi.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#define TAG "SPI"

void spi_init() {
    spi_bus_config_t busconfig = {
        .miso_io_num = 34,
        .mosi_io_num = 33,
        .sclk_io_num = 32,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
        .max_transfer_sz = 32,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &busconfig, SPI2_HOST));
}
