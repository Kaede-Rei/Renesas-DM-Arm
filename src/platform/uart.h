#ifndef _uart_h_
#define _uart_h_

#include <stdbool.h>
#include "infra/protocol_parser.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief UART 单例用户自定义名称
 */
#define uart uart_instance

/**
 * @brief UART 错误代码枚举类型
 * @param UART_SUCCESS 操作成功
 * @param UART_ERROR 操作失败
 */
typedef enum {
    UART_SUCCESS = 0,   ///< 操作成功
    UART_ERROR,         ///< 操作失败
} UartErrorCode_e;

/**
 * @brief UART 端口枚举类型
 * @param UART6 UART6 端口
 * @param UART7 UART7 端口
 */
typedef enum {
    UART_START = 0,
    UART6,
    UART7,
    UART_END
} Uart_te;

/**
 * @brief UART 单例接口
 * @param init 初始化 UART 的函数指针
 * @param write 发送数据的函数指针
 * @param read 接收数据的函数指针
 * @param wait_tx_complete 等待发送完成的函数指针
 * @param wait_rx_complete 等待接收完成的函数指针
 * @param is_tx_complete 检查发送是否完成的函数指针
 * @param is_rx_complete 检查接收是否完成的函数指针
 */
extern const struct UartInterface {
    /**
     * @brief 初始化指定 UART 设备
     * @param uart 指定的 UART 设备（Uart_te 枚举类型）
     * @param rx_buf 接收缓冲区指针
     * @return UartErrorCode_e 枚举类型，表示操作结果
     */
    UartErrorCode_e(*init)(Uart_te uart, struct RingBuf* rx_buf);

    /**
     * @brief 发送数据
     * @param uart 指定的 UART 设备
     * @param data 要发送的数据指针
     * @param length 数据长度
     * @return UartErrorCode_e 枚举类型，表示操作结果
     */
    UartErrorCode_e(*write)(Uart_te uart, const uint8_t* data, uint16_t length);

    /**
     * @brief 接收数据
     * @param uart 指定的 UART 设备
     * @param data 接收数据的缓冲区指针
     * @param length 数据长度
     * @return UartErrorCode_e 枚举类型，表示操作结果
     */
    UartErrorCode_e(*read)(Uart_te uart, uint8_t* data, uint16_t length);

    /**
     * @brief 等待发送完成
     * @param uart 指定的 UART 设备
     * @return UartErrorCode_e 枚举类型，表示操作结果
     */
    UartErrorCode_e(*wait_tx_complete)(Uart_te uart);

    /**
     * @brief 等待接收完成
     * @param uart 指定的 UART 设备
     * @return UartErrorCode_e 枚举类型，表示操作结果
     */
    UartErrorCode_e(*wait_rx_complete)(Uart_te uart);

    /**
     * @brief 检查发送是否完成
     * @param uart 指定的 UART 设备
     * @return true 表示完成，false 未完成
     */
    bool (*is_tx_complete)(Uart_te uart);

    /**
     * @brief 检查接收是否完成
     * @param uart 指定的 UART 设备
     * @return true 表示完成，false 未完成
     */
    bool (*is_rx_complete)(Uart_te uart);
} uart_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

UartErrorCode_e uart_init(Uart_te uart, struct RingBuf* rx_buf);
UartErrorCode_e uart_write(Uart_te uart, const uint8_t* data, uint16_t length);
UartErrorCode_e uart_read(Uart_te uart, uint8_t* data, uint16_t length);
UartErrorCode_e uart_wait_tx_complete(Uart_te uart);
UartErrorCode_e uart_wait_rx_complete(Uart_te uart);
bool uart_is_tx_complete(Uart_te uart);
bool uart_is_rx_complete(Uart_te uart);

#endif
