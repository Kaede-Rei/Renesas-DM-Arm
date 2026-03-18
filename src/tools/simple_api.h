#ifndef _simple_api_h_
#define _simple_api_h_

#include <stdio.h>
#include <math.h>

// ! ========================= GPIO / IOPORT 简 化 宏 ========================= ! //

static inline void printf_float(float val) {
    unsigned int int_part = (unsigned int)val;
    unsigned int frac_part = (unsigned int)(fabs(val - (float)int_part) * 1000);
    if(val >= 0) printf("%d.%03d", int_part, frac_part);
    else printf("-%d.%03d", int_part, frac_part);
}

static inline void printf_double(double val) {
    unsigned int int_part = (unsigned int)val;
    unsigned int frac_part = (unsigned int)(fabs(val - (double)int_part) * 1000);
    if(val >= 0) printf("%d.%03d", int_part, frac_part);
    else printf("-%d.%03d", int_part, frac_part);
}

/**
 * @brief 读取引脚电平
 * @param port_pin 引脚编号，例如 BSP_IO_PORT_00_PIN_00
 * @param level 指向 bsp_io_level_t 变量的指针，用于存储读取到的电平值
 * @return fsp_err_t 表示操作结果
 * @note 示例：bsp_io_level_t level; gpio_read(BSP_IO_PORT_00_PIN_00, &level);
 */
#define gpio_read(port_pin, level) g_ioport.p_api->pinRead(&g_ioport_ctrl, (port_pin), (level))

/**
 * @brief 写入引脚电平
 * @param port_pin 引脚编号，例如 BSP_IO_PORT_04_PIN_00
 * @param level 要写入的电平值（BSP_IO_LEVEL_HIGH 或 BSP_IO_LEVEL_LOW）
 * @return fsp_err_t 表示操作结果
 * @note 示例：gpio_write(BSP_IO_PORT_04_PIN_00, BSP_IO_LEVEL_HIGH);
 */
#define gpio_write(port_pin, level) g_ioport.p_api->pinWrite(&g_ioport_ctrl, (port_pin), (level))

/**
 * @brief 配置单个引脚属性
 * @param port_pin 引脚编号，例如 BSP_IO_PORT_01_PIN_05
 * @param cfg 引脚配置值（bsp_io_port_pin_cfg_t 类型，可使用 | 组合）
 * @return fsp_err_t 表示操作结果
 * @note 示例：
 *       gpio_cfg(BSP_IO_PORT_01_PIN_05, BSP_IO_DIRECTION_OUTPUT | BSP_IO_PULLUP_ENABLE);
 */
#define gpio_cfg(port_pin, cfg) g_ioport.p_api->pinCfg(&g_ioport_ctrl, (port_pin), (cfg))

/**
* @brief 翻转指定引脚电平（高↔低）
* @param port_pin 引脚编号，例如 BSP_IO_PORT_02_PIN_10
* @return fsp_err_t 表示操作结果
* @note 该宏会先读取当前电平，再写入相反值
* @note 示例：gpio_toggle(BSP_IO_PORT_02_PIN_10);
*/
#define gpio_toggle(port_pin) \
    do { \
        bsp_io_level_t __lvl; \
        if (FSP_SUCCESS == g_ioport.p_api->pinRead(&g_ioport_ctrl, (port_pin), &__lvl)) { \
            g_ioport.p_api->pinWrite(&g_ioport_ctrl, (port_pin), !__lvl); \
        } \
    } while (0)

/**
 * @brief 同时配置多个引脚（批量配置）
 * @param p_pins_cfg 指向 bsp_io_port_pin_cfg_t 数组的指针
 * @param num_pins 要配置的引脚数量
 * @return fsp_err_t 表示操作结果
 * @note 示例：
 *       const bsp_io_port_pin_cfg_t pins[] = {
 *           {BSP_IO_PORT_03_PIN_04, BSP_IO_DIRECTION_OUTPUT | BSP_IO_LEVEL_HIGH},
 *           {BSP_IO_PORT_03_PIN_05, BSP_IO_DIRECTION_INPUT}
 *       };
 *       gpio_cfg_pins(pins, sizeof(pins)/sizeof(pins[0]));
 */
#define gpio_cfg_pins(p_pins_cfg, num_pins) g_ioport.p_api->pinsCfg(&g_ioport_ctrl, (p_pins_cfg), (num_pins))

/**
 * @brief 读取整个端口的输入状态（16位或8位端口）
 * @param port 端口编号，例如 BSP_IO_PORT_00_PIN
 * @param p_value 指向 uint16_t 或 uint8_t 的指针，用于存储端口值
 * @return fsp_err_t 表示操作结果
 * @note 示例：uint16_t port_val; gpio_port_read(BSP_IO_PORT_00_PIN, &port_val);
 */
#define gpio_port_read(port, p_value) g_ioport.p_api->portRead(&g_ioport_ctrl, (port), (p_value))

/**
 * @brief 向整个端口写入值（适用于输出端口）
 * @param port 端口编号，例如 BSP_IO_PORT_04_PIN
 * @param value 要写入的16位或8位值
 * @return fsp_err_t 表示操作结果
 * @note 示例：gpio_port_write(BSP_IO_PORT_04_PIN, 0x00FF);
 */
#define gpio_port_write(port, value) g_ioport.p_api->portWrite(&g_ioport_ctrl, (port), (value))

#endif
