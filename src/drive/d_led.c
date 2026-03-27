
#include "d_led.h"
#include "hal_data.h"   // IWYU pragma: keep
#include "tools/simple_api.h"

// ! ========================= 变 量 声 明 ========================= ! //

const struct LedInterface d_led_instance = {
    .on = d_led_on,
    .off = d_led_off,
    .toggle = d_led_toggle
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

LedErrorCode_e d_led_on(void) {
    gpio_write(BSP_IO_PORT_04_PIN_00, BSP_IO_LEVEL_LOW);
    return LED_SUCCESS;
}

LedErrorCode_e d_led_off(void) {
    gpio_write(BSP_IO_PORT_04_PIN_00, BSP_IO_LEVEL_HIGH);
    return LED_SUCCESS;
}

LedErrorCode_e d_led_toggle(void) {
    gpio_toggle(BSP_IO_PORT_04_PIN_00);
    return LED_SUCCESS;
}


