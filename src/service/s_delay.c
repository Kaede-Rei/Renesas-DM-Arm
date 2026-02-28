/**
 * @file    s_delay.c
 * @brief   延时服务实现
 */
#include "s_delay.h"

// ! ========================= 变 量 声 明 ========================= ! //

typedef struct {
    ms_t(*get_ms)(void);
    bool(*ms_timeout)(ms_t start, uint32_t timeout_ms);
} delay_ops_t;

static delay_ops_t _delay_ops = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief   延时服务初始化
 * @param   get_ms 获取当前毫秒数的函数指针
 * @param   ms_timeout 判断毫秒级是否超时的函数指针, 超时返回 true, 未超时返回 false
 * @param   get_us 获取当前微秒数的函数指针
 * @param   us_timeout 判断微秒级是否超时的函数指针, 超时返回 true, 未超时返回 false
 * @note    用法: 在系统初始化时调用, 传入对应的函数指针, 即可使用延时服务. 例如:
 * @note    s_delay_init(systick_get_ms, systick_is_timeout, dwt_get_us, dwt_is_timeout);
 * @note    其中: ms_t = us_t = uint32_t
 */
void s_delay_init(ms_t(*get_ms)(void), bool(*ms_timeout)(ms_t start, ms_t timeout_ms)) {
    _delay_ops.get_ms = get_ms;
    _delay_ops.ms_timeout = ms_timeout;
}

/**
 * @brief   毫秒级阻塞延时
 * @param   ms 延时毫秒数
 * @retval  None
 */
void s_delay_ms(ms_t ms) {
    ms_t start = _delay_ops.get_ms();
    while(!_delay_ops.ms_timeout(start, ms));
}

/**
 * @brief   秒级阻塞延时
 * @param   s 延时秒数
 * @retval  None
 */
void s_delay_s(ms_t s) {
    ms_t start = _delay_ops.get_ms();
    while(!_delay_ops.ms_timeout(start, s * 1000));
}

/**
 * @brief   非阻塞毫秒延时
 * @param   start 指向起始时间的指针
 * @param   interval_ms 延时间隔(ms)
 * @retval  bool -  true:时间到, false:未到
 */
bool s_nb_delay_ms(ms_t* start, ms_t interval_ms) {
    if(*start == 0) {
        *start = _delay_ops.get_ms();
        return false;
    }
    if(_delay_ops.ms_timeout(*start, interval_ms)) {
        *start = 0;
        return true;
    }
    return false;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //


