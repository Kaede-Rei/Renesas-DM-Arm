
#ifndef _led_h_
#define _led_h_

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief LED 单例用户自定义名称
 */
#define led led_instance

/**
 * @brief LED 错误代码枚举类型
 * @param LED_SUCCESS 操作成功
 * @param LED_ERROR 操作失败
 */
typedef enum {
    LED_SUCCESS = 0,   ///< 操作成功
    LED_ERROR,         ///< 操作失败
} LedErrorCode_e;

/**
 * @brief LED 单例接口
 * @param on 打开 LED 的函数指针
 * @param off 关闭 LED 的函数指针
 * @param toggle 翻转 LED 状态的函数指针
 */
extern const struct LedInterface {
    /**
     * @brief 打开 LED
     * @return LedErrorCode_e 枚举类型，表示操作结果
     */
    LedErrorCode_e(*on)(void);

    /**
     * @brief 关闭 LED
     * @return LedErrorCode_e 枚举类型，表示操作结果
     */
    LedErrorCode_e(*off)(void);

    /**
     * @brief 翻转 LED 状态
     * @return LedErrorCode_e 枚举类型，表示操作结果
     */
    LedErrorCode_e(*toggle)(void);
} led_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

LedErrorCode_e led_on(void);
LedErrorCode_e led_off(void);
LedErrorCode_e led_toggle(void);

#endif