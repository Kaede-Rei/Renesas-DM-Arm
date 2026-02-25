#ifndef _callback_h_
#define _callback_h_

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

typedef enum{
    UART_START = 0,
    UART7,
    UART_END
} Uart_te;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void uart_wait_tx_complete(Uart_te uart);
void uart_wait_rx_complete(Uart_te uart);

#endif
