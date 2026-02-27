#include "d_callback.h"

#include "hal_data.h"

// ! ========================= 变 量 声 明 ========================= ! //

// UART7 标志符
static volatile struct {
    uint8_t tx_7;
    uint8_t rx_7;
} uart_complete_flags = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 等待指定 UART 发送完成
 * @param uart 指定的 UART 设备（Uart_te 枚举类型）
 * @return 无
 * @note 示例：uart_wait_tx_complete(UART7);
 */
void uart_wait_tx_complete(Uart_te uart) {
    switch(uart) {
        case UART7:
            while(!uart_complete_flags.tx_7);
            uart_complete_flags.tx_7 = 0;
            break;
        default:
            break;
    }
}

/**
 * @brief 等待指定 UART 接收完成
 * @param uart 指定的 UART 设备（Uart_te 枚举类型）
 * @return 无
 * @note 示例：uart_wait_rx_complete(UART7);
 */
void uart_wait_rx_complete(Uart_te uart) {
    switch(uart) {
        case UART7:
            while(!uart_complete_flags.rx_7);
            uart_complete_flags.rx_7 = 0;
            break;
        default:
            break;
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

void uart7_callback(uart_callback_args_t* p_args) {
    switch(p_args->event) {
        case UART_EVENT_TX_COMPLETE:
            uart_complete_flags.tx_7 = 1;
            break;
        case UART_EVENT_RX_COMPLETE:
            uart_complete_flags.rx_7 = 1;
            break;
        default:
            break;
    }
}
