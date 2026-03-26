#ifndef _systick_h_
#define _systick_h_

#include "hal_data.h"   // IWYU pragma: keep

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

#define systick systick_instance

typedef uint32_t ms_t;

extern const struct SystickInstance {
    /**
     * @brief 初始化 SysTick 定时器
     * @return fsp_err_t 枚举类型，表示操作结果
     */
    fsp_err_t(*init)(void);
    /**
    * @brief 获取系统运行的毫秒数
    * @return 当前系统运行的毫秒数
    */
    ms_t(*get_ms)(void);
    /**
     * @brief 获取系统运行的秒数
     * @return 当前系统运行的秒数
     */
    ms_t(*get_s)(void);
    /**
     * @brief 判断是否达到指定的毫秒级超时时间
     * @param start 起始时间（毫秒）
     * @param timeout_ms 超时时间（毫秒）
     * @return true 表示已超时，false 表示未超时
     */
    bool (*is_timeout)(ms_t start, ms_t timeout_ms);
} systick_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

fsp_err_t systick_init(void);
ms_t systick_get_ms(void);
ms_t systick_get_s(void);
bool systick_is_timeout(ms_t start, ms_t timeout_ms);

#endif
