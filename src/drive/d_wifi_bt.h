#ifndef _d_wifi_bt_h_
#define _d_wifi_bt_h_

#include "service/s_delay.h"
#include "d_uart.h"
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

typedef enum {
    WIFI_BT_SUCCESS = 0,
    WIFI_BT_ERROR,
    WIFI_BT_WAIT_AP,
    WIFI_BT_TIMEOUT,
    WIFI_BT_PROCESSING,
    WIFI_BT_FRAME_READY,
    WIFI_BT_NO_FRAME,
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
    const char* ssid;
    const char* password;

    NetworkProtocol protocol;
    LocalRole role;
    char ip[16];
    uint16_t remote_port;
    uint16_t local_port;
    uint16_t socket_port;
} WifiBtConnectInfo;

#define WIFI_BT_FRAME_RX_BUF_SIZE   512
#define WIFI_BT_FRAME_BUF_SIZE      256

// ! ========================= 接 口 函 数 声 明 ========================= ! //

WifiBtErrorCode d_wifi_bt_init(Uart_te uart, WifiBtWorkMode mode, const uint8_t* const header, const uint8_t header_len);
WifiBtErrorCode d_wifi_bt_send_cmd(const char* cmd);
WifiBtErrorCode d_wifi_bt_join_ap(const char* ssid, const char* password);
WifiBtErrorCode d_wifi_bt_rejoin_ap(const char* ssid, const char* password);
WifiBtErrorCode d_wifi_bt_check_ap(void);
WifiBtErrorCode d_wifi_bt_connect(WifiBtConnectInfo* info, ms_t timeout_ms);
WifiBtErrorCode d_wifi_bt_disconnect(WifiBtConnectInfo info);
WifiBtErrorCode d_wifi_bt_reset(WifiBtWorkMode mode);

WifiBtErrorCode d_wifi_bt_heartbeat(WifiBtConnectInfo* info, ms_t timeout_ms);
WifiBtErrorCode d_wifi_bt_process(uint8_t** const frame_buf, uint16_t* const frame_len);
WifiBtErrorCode d_wifi_bt_send_frame(WifiBtConnectInfo info, const uint8_t* data, uint16_t length);
void d_wifi_bt_finish_frame(void);

#endif
