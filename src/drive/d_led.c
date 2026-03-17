#include "d_led.h"

#include "hal_data.h"
#include "tools/simple_api.h"

// ! ========================= 变 量 声 明 ========================= ! //



// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //

void d_led_on() {
    gpio_write(BSP_IO_PORT_04_PIN_00, BSP_IO_LEVEL_LOW);
}

void d_led_off() {
    gpio_write(BSP_IO_PORT_04_PIN_00, BSP_IO_LEVEL_HIGH);
}

void d_led_toggle() {
    gpio_toggle(BSP_IO_PORT_04_PIN_00);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //


