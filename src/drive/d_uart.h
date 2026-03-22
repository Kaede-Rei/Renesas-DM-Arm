#ifndef _d_uart_h_
#define _d_uart_h_

#include <stdbool.h>

#include "tools/protocol_parser.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

typedef enum {
    UART_START = 0,
    UART6,
    UART7,
    UART_END
} Uart_te;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void d_uart_init(Uart_te uart, struct RingBuf* rx_buf);
void d_uart_write(Uart_te uart, const uint8_t* data, uint16_t length);
void d_uart_read(Uart_te uart, uint8_t* data, uint16_t length);
void d_uart_wait_tx_complete(Uart_te uart);
void d_uart_wait_rx_complete(Uart_te uart);
bool d_uart_is_tx_complete(Uart_te uart);
bool d_uart_is_rx_complete(Uart_te uart);

#endif
