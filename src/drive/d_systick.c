#include "d_systick.h"


// ! ========================= 变 量 声 明 ========================= ! //

const struct SystickInstance d_systick_instance = {
    .init = d_systick_init,
    .get_ms = d_systick_get_ms,
    .get_s = d_systick_get_s,
    .is_timeout = d_systick_is_timeout
};

static volatile ms_t _ms = 0;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

void SysTick_Handler(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化 SysTick 定时器
 * @return fsp_err_t 枚举类型，表示操作结果
 */
fsp_err_t d_systick_init(void) {
    uint32_t uw_sysclk = R_BSP_SourceClockHzGet(FSP_PRIV_CLOCK_PLL);

    if(SysTick_Config(uw_sysclk / 1000) != 0) {
        return FSP_ERR_ASSERTION;
    }

    return FSP_SUCCESS;
}

/**
 * @brief 获取系统运行的毫秒数
 * @return 当前系统运行的毫秒数
 */
ms_t d_systick_get_ms(void) {
    return _ms;
}

/**
 * @brief 获取系统运行的秒数
 * @return 当前系统运行的秒数
 */
ms_t d_systick_get_s(void) {
    return _ms / 1000;
}

/**
 * @brief 判断是否达到指定的毫秒级超时时间
 * @param start 起始时间（毫秒）
 * @param timeout_ms 超时时间（毫秒）
 * @return true 表示已超时，false 表示未超时
 */
bool d_systick_is_timeout(ms_t start, ms_t timeout_ms) {
    ms_t now = d_systick_get_ms();
    if(now >= start) {
        return (now - start) >= timeout_ms ? true : false;
    }
    else {
        return ((0xFFFFFFFFU - start) + now + 1) >= timeout_ms ? true : false;
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief   SysTick 中断服务
 * @param   None
 * @retval  None
 */
void SysTick_Handler(void) {
    _ms++;
}
