#include "d_uart.h"

#include "hal_data.h"
#include "tools/protocol_parser.h"

#include <stdint.h>
#include <stdio.h>

void protocol_parser_enter_critical() {
    __disable_irq();
}

void protocol_parser_exit_critical() {
    __enable_irq();
}


// ! ========================= 变 量 声 明 ========================= ! //

// 单例结构体实现
const struct UartInterface d_uart_instance = {
    .init = d_uart_init,
    .write = d_uart_write,
    .read = d_uart_read,
    .wait_tx_complete = d_uart_wait_tx_complete,
    .wait_rx_complete = d_uart_wait_rx_complete,
    .is_tx_complete = d_uart_is_tx_complete,
    .is_rx_complete = d_uart_is_rx_complete
};

typedef struct {
    Uart_te uart;
    uint8_t rx_byte;
    RingBuf* rx_buf;
} UartConfig_t;

// UART 标志符
static volatile struct {
    uint8_t tx_6;
    uint8_t rx_6;
    uint8_t tx_7;
    uint8_t rx_7;
} uart_complete_flags = { 0 };

UartConfig_t uart6_config = {
    .uart = UART6,
    .rx_byte = 0,
    .rx_buf = NULL
};
UartConfig_t uart7_config = {
    .uart = UART7,
    .rx_byte = 0,
    .rx_buf = NULL
};

// ! ========================= 私 有 函 数 声 明 ========================= ! ///



// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化指定 UART 设备
 * @param uart_instance 指定的 UART 设备（Uart_te 枚举类型）
 * @param rx_buffer 接收缓冲区指针
 * @return UartErrorCode_e 枚举类型，表示操作结果
 */

UartErrorCode_e d_uart_init(Uart_te uart_instance, RingBuf* rx_buf) {
    switch(uart_instance) {
        case UART6:
            uart6_config.rx_buf = rx_buf;
            g_uart6.p_api->open(g_uart6.p_ctrl, g_uart6.p_cfg);
            g_uart6.p_api->read(g_uart6.p_ctrl, (uint8_t* const)&uart6_config.rx_byte, 1);
            return UART_SUCCESS;
        case UART7:
            uart7_config.rx_buf = rx_buf;
            g_uart7.p_api->open(g_uart7.p_ctrl, g_uart7.p_cfg);
            g_uart7.p_api->read(g_uart7.p_ctrl, (uint8_t* const)&uart7_config.rx_byte, 1);
            return UART_SUCCESS;
        default:
            return UART_ERROR;
    }
}

/**
 * @brief 等待指定 UART 发送完成
 * @param uart_instance 指定的 UART 设备（Uart_te 枚举类型）
 * @return UartErrorCode_e 枚举类型，表示操作结果
 * @note 示例：d_uart_wait_tx_complete(UART7);
 */
UartErrorCode_e d_uart_wait_tx_complete(Uart_te uart_instance) {
    switch(uart_instance) {
        case UART6:
            while(!uart_complete_flags.tx_6);
            uart_complete_flags.tx_6 = 0;
            return UART_SUCCESS;
        case UART7:
            while(!uart_complete_flags.tx_7);
            uart_complete_flags.tx_7 = 0;
            return UART_SUCCESS;
        default:
            return UART_ERROR;
    }
}

/**
 * @brief 等待指定 UART 接收完成
 * @param uart_instance 指定的 UART 设备（Uart_te 枚举类型）
 * @return UartErrorCode_e 枚举类型，表示操作结果
 * @note 示例：d_uart_wait_rx_complete(UART7);
 */
UartErrorCode_e d_uart_wait_rx_complete(Uart_te uart_instance) {
    switch(uart_instance) {
        case UART6:
            while(!uart_complete_flags.rx_6);
            uart_complete_flags.rx_6 = 0;
            return UART_SUCCESS;
        case UART7:
            while(!uart_complete_flags.rx_7);
            uart_complete_flags.rx_7 = 0;
            return UART_SUCCESS;
        default:
            return UART_ERROR;
    }
}

/**
 * @brief 判断指定 UART 是否发送完成
 * @param uart_instance 指定的 UART 设备（Uart_te 枚举类型）
 * @return 布尔值，true 表示发送完成，false 表示未完成
 */
bool d_uart_is_tx_complete(Uart_te uart_instance) {
    switch(uart_instance) {
        case UART6:
            if(!uart_complete_flags.tx_6) return false;
            uart_complete_flags.tx_6 = 0;
            return true;
        case UART7:
            if(!uart_complete_flags.tx_7) return false;
            uart_complete_flags.tx_7 = 0;
            return true;
        default:
            return false;
    }
}

/**
 * @brief 判断指定 UART 是否接收完成
 * @param uart_instance 指定的 UART 设备（Uart_te 枚举类型）
 * @return 布尔值，true 表示接收完成，false 表示未完成
 */
bool d_uart_is_rx_complete(Uart_te uart_instance) {
    switch(uart_instance) {
        case UART6:
            if(!uart_complete_flags.rx_6) return false;
            uart_complete_flags.rx_6 = 0;
            return true;
        case UART7:
            if(!uart_complete_flags.rx_7) return false;
            uart_complete_flags.rx_7 = 0;
            return true;
        default:
            return false;
    }
}

/**
 * @brief 向指定 UART 发送数据
 * @param uart_instance 指定的 UART 设备（Uart_te 枚举类型）
 * @param data 要发送的数据指针
 * @param length 要发送的数据长度
 * @return UartErrorCode_e 枚举类型，表示操作结果
 */
UartErrorCode_e d_uart_write(Uart_te uart_instance, const uint8_t* data, uint16_t length) {
    switch(uart_instance) {
        case UART6:
            g_uart6.p_api->write(g_uart6.p_ctrl, data, length);
            return UART_SUCCESS;
        case UART7:
            g_uart7.p_api->write(g_uart7.p_ctrl, data, length);
            return UART_SUCCESS;
        default:
            return UART_ERROR;
    }
}

/**
 * @brief 从指定 UART 接收数据
 * @param uart_instance 指定的 UART 设备（Uart_te 枚举类型）
 * @param data 存储接收数据的缓冲区指针
 * @param length 要接收的数据长度
 * @return UartErrorCode_e 枚举类型，表示操作结果
 */
UartErrorCode_e d_uart_read(Uart_te uart_instance, uint8_t* data, uint16_t length) {
    uint8_t* temp = data;
    switch(uart_instance) {
        case UART6:
            while(length--) {
                if(uart6_config.rx_buf->is_empty(uart6_config.rx_buf)) break;
                uart6_config.rx_buf->read(uart6_config.rx_buf, temp++);
            }
            return UART_SUCCESS;
        case UART7:
            while(length--) {
                if(uart7_config.rx_buf->is_empty(uart7_config.rx_buf)) break;
                uart7_config.rx_buf->read(uart7_config.rx_buf, temp++);
            }
            return UART_SUCCESS;
        default:
            return UART_ERROR;
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

void uart6_callback(uart_callback_args_t* p_args) {
    switch(p_args->event) {
        case UART_EVENT_TX_COMPLETE:
            uart_complete_flags.tx_6 = 1;
            break;
        case UART_EVENT_RX_COMPLETE:
            uart_complete_flags.rx_6 = 1;
            if(uart6_config.rx_buf) uart6_config.rx_buf->write(uart6_config.rx_buf, uart6_config.rx_byte);
            g_uart6.p_api->read(g_uart6.p_ctrl, (uint8_t* const)&uart6_config.rx_byte, 1);
            break;
        default:
            break;
    }
}

void uart7_callback(uart_callback_args_t* p_args) {
    switch(p_args->event) {
        case UART_EVENT_TX_COMPLETE:
            uart_complete_flags.tx_7 = 1;

            break;
        case UART_EVENT_RX_COMPLETE:
            uart_complete_flags.rx_7 = 1;

            if(uart7_config.rx_buf) uart7_config.rx_buf->write(uart7_config.rx_buf, uart7_config.rx_byte);
            g_uart7.p_api->read(g_uart7.p_ctrl, (uint8_t* const)&uart7_config.rx_byte, 1);

            break;
        default:
            break;
    }
}
