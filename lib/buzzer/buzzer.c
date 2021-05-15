#include "driver/gpio.h"

static int pin;

void buzzer_init(int gpio_num) {
    pin = gpio_num;
    gpio_config_t conf = {
        .pin_bit_mask = (0x01 << pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&conf);
}

void buzzer_on() {
    gpio_set_level(pin, 1);
}

void buzzer_off() {
    gpio_set_level(pin, 0);
}