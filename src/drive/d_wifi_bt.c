#include "d_wifi_bt.h"

#include "service/s_delay.h"
#include "drive/d_systick.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// ! ========================= 变 量 声 明 ========================= ! //

static struct {
    Uart_te uart;
    RingBuf* rx_buf;
} wifi_bt_config = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void wifi_bt_send_raw(const char* str);
static bool wifi_bt_wait_str(const char* expected, uint8_t length, uint32_t timeout_ms);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

WifiBtErrorCode d_wifi_bt_init(Uart_te uart, RingBuf* rx_buf, WifiBtWorkMode mode) {
    wifi_bt_config.uart = uart;
    wifi_bt_config.rx_buf = rx_buf;
    d_uart_init(uart, rx_buf);

    s_delay_ms(10);
    char str[12];
    snprintf(str, sizeof(str), "AT+WPRT=%d\r\n", mode);
    wifi_bt_send_raw(str);
    if(!wifi_bt_wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;

    return WIFI_BT_SUCCESS;
}

WifiBtErrorCode d_wifi_bt_send_cmd(const char* cmd) {
    char buf[64];
    snprintf(buf, sizeof(buf), "%s\r\n", cmd);

    wifi_bt_send_raw(buf);

    return WIFI_BT_SUCCESS;
}

WifiBtErrorCode d_wifi_bt_join_ap(const char* ssid, const char* pwd) {
    char cmd[128];

    snprintf(cmd, sizeof(cmd), "AT+SSID=%s", ssid);
    if(d_wifi_bt_send_cmd(cmd) != WIFI_BT_SUCCESS) return WIFI_BT_ERROR;
    if(!wifi_bt_wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;

    snprintf(cmd, sizeof(cmd), "AT+KEY=1,0,%s", pwd);
    if(d_wifi_bt_send_cmd(cmd) != WIFI_BT_SUCCESS) return WIFI_BT_ERROR;
    if(!wifi_bt_wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;

    snprintf(cmd, sizeof(cmd), "AT+WJOIN");
    if(d_wifi_bt_send_cmd(cmd) != WIFI_BT_SUCCESS) return WIFI_BT_ERROR;
    if(!wifi_bt_wait_str("+OK", 3, 10000)) return WIFI_BT_ERROR;

    return WIFI_BT_SUCCESS;
}

WifiBtErrorCode d_wifi_bt_connect(WifiBtConnectInfo* info, uint32_t timeout_ms) {
    char cmd[128];

    snprintf(cmd, sizeof(cmd), "AT+SKCT=%d,%d,\"%s\",%d,%d",
        info->protocol, info->role, info->ip, info->remote_port, info->local_port);
    if(d_wifi_bt_send_cmd(cmd) != WIFI_BT_SUCCESS) return WIFI_BT_ERROR;
    if(!wifi_bt_wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;

    char buf[32] = { 0 };
    uint8_t idx = 0;
    char ch;
    ms_t start_time = d_systick_get_ms();
    while(d_systick_get_ms() - start_time < timeout_ms) {
        if(!wifi_bt_config.rx_buf->is_empty(wifi_bt_config.rx_buf)) {
            wifi_bt_config.rx_buf->read(wifi_bt_config.rx_buf, (uint8_t*)&ch);
            buf[idx++] = ch;
            if(strstr(buf, "\r\n")) break;
            else if(idx >= sizeof(buf) - 1) return WIFI_BT_ERROR;
        }
        else s_delay_ms(1);
    }

    info->socket_id = 0;
    for(idx = 0; buf[idx] != '\r' && idx < sizeof(buf); ++idx) {
        if(buf[idx] >= '0' && buf[idx] <= '9') {
            info->socket_id = (uint16_t)(info->socket_id * 10 + (buf[idx] - '0'));
        }
    }
    printf("IP:%s - SocketPort:%d\r\n", info->ip, info->socket_id);

    return WIFI_BT_SUCCESS;
}

WifiBtErrorCode d_wifi_bt_disconnect(WifiBtConnectInfo info) {
    char cmd[128];

    snprintf(cmd, sizeof(cmd), "AT+SKCLS=%d\r\n", info.socket_id);
    if(d_wifi_bt_send_cmd(cmd) != WIFI_BT_SUCCESS) return WIFI_BT_ERROR;
    if(!wifi_bt_wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;

    return WIFI_BT_SUCCESS;
}

WifiBtErrorCode d_wifi_bt_send(WifiBtConnectInfo info, const uint8_t* data, uint16_t length) {
    char cmd[128];

    snprintf(cmd, sizeof(cmd), "AT+SKSND=%d,%d", info.socket_id, length);
    if(d_wifi_bt_send_cmd(cmd) != WIFI_BT_SUCCESS) return WIFI_BT_ERROR;
    if(!wifi_bt_wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;
    d_uart_write(wifi_bt_config.uart, data, length);
    d_uart_wait_tx_complete(wifi_bt_config.uart);

    return WIFI_BT_SUCCESS;
}


// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void wifi_bt_send_raw(const char* str) {
    d_uart_write(wifi_bt_config.uart, (uint8_t*)str, (uint16_t)strlen(str));
    d_uart_wait_tx_complete(wifi_bt_config.uart);
}

static bool wifi_bt_wait_str(const char* expected, uint8_t length, uint32_t timeout_ms) {
    int idx = 0;
    char ch;
    ms_t start_time = d_systick_get_ms();

    while(d_systick_get_ms() - start_time < timeout_ms) {
        if(!wifi_bt_config.rx_buf->is_empty(wifi_bt_config.rx_buf)) {
            wifi_bt_config.rx_buf->read(wifi_bt_config.rx_buf, (uint8_t*)&ch);
            if(ch == expected[idx]) {
                if(++idx == length)
                    return true;
            }
            else idx = 0;
        }
        else s_delay_ms(1);
    }

    return false;
}