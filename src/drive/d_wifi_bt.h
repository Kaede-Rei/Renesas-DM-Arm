#ifndef _d_wifi_bt_h_
#define _d_wifi_bt_h_

#include "d_uart.h"
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

typedef enum {
    WIFI_BT_SUCCESS = 0,
    WIFI_BT_ERROR,
    WIFI_BT_TIMEOUT,
} WifiBtErrorCode;

typedef enum {
    STA = 0,
    SOFT_AP = 2,
    AP_STA = 3,
} WifiBtWorkMode;

typedef enum {
    TCP = 0,
    UDP = 1,
} NetworkProtocol;

typedef enum {
    Client = 0,
    Server = 1,
} LocalRole;

typedef struct {
    NetworkProtocol protocol;
    LocalRole role;
    char ip[16];
    uint16_t remote_port;
    uint16_t local_port;
    uint16_t socket_port;
} WifiBtConnectInfo;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

WifiBtErrorCode d_wifi_bt_init(Uart_te uart, RingBuf* rx_buf, WifiBtWorkMode mode);
WifiBtErrorCode d_wifi_bt_send_cmd(const char* cmd);
WifiBtErrorCode d_wifi_bt_join_ap(const char* ssid, const char* password);
WifiBtErrorCode d_wifi_bt_connect(WifiBtConnectInfo* info, uint32_t timeout_ms);
WifiBtErrorCode d_wifi_bt_disconnect(WifiBtConnectInfo info);
WifiBtErrorCode d_wifi_bt_send(WifiBtConnectInfo info, const uint8_t* data, uint16_t length);
WifiBtErrorCode d_wifi_bt_recv(WifiBtConnectInfo info, uint8_t* buffer, uint16_t max_length, uint32_t timeout_ms);

#endif
