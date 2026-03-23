#ifndef _d_wifi_bt_h_
#define _d_wifi_bt_h_

#include "service/s_delay.h"
#include "d_uart.h"

#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief WiFi/BT 模块错误码
 * @param WIFI_BT_SUCCESS 成功
 * @param WIFI_BT_ERROR 错误
 * @param WIFI_BT_WAIT_AP 等待连接 AP
 * @param WIFI_BT_TIMEOUT 超时
 * @param WIFI_BT_PROCESSING 处理中
 * @param WIFI_BT_FRAME_READY 有帧可处理
 * @param WIFI_BT_NO_FRAME 没有帧可处理
 */
typedef enum {
    WIFI_BT_SUCCESS = 0,
    WIFI_BT_ERROR,
    WIFI_BT_WAIT_AP,
    WIFI_BT_TIMEOUT,
    WIFI_BT_PROCESSING,
    WIFI_BT_FRAME_READY,
    WIFI_BT_NO_FRAME,
} WifiBtErrorCode;

/**
 * @brief WiFi/BT 模块工作模式
 * @param STA 站点模式
 * @param SOFT_AP 软 AP 模式
 * @param AP_STA AP + 站点模式
 */
typedef enum {
    STA = 0,
    SOFT_AP = 2,
    AP_STA = 3,
} WifiBtWorkMode;

/**
 * @brief 网络协议类型
 * @param TCP TCP 协议
 * @param UDP UDP 协议
 */
typedef enum {
    TCP = 0,
    UDP = 1,
} NetworkProtocol;

/**
 * @brief 本地角色
 * @param Client 客户端
 * @param Server 服务器
 */
typedef enum {
    Client = 0,
    Server = 1,
} LocalRole;

/**
 * @brief WiFi/BT 连接信息结构体
 * @param ssid WiFi SSID
 * @param password WiFi 密码
 * @param protocol 网络协议类型
 * @param role 本地角色
 * @param ip 远程 IP 地址
 * @param remote_port 远程端口
 * @param local_port 本地端口
 * @param socket_port 套接字端口
 */
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

/// @brief WiFi/BT 模块帧缓冲区大小
#define WIFI_BT_FRAME_RX_BUF_SIZE   512
/// @brief WiFi/BT 模块帧发送缓冲区大小
#define WIFI_BT_FRAME_BUF_SIZE      256

// ! ========================= 接 口 函 数 声 明 ========================= ! //

WifiBtErrorCode d_wifi_bt_init(Uart_te uart, WifiBtWorkMode mode, const uint8_t* const header, const uint8_t header_len);
WifiBtErrorCode d_wifi_bt_send_cmd(const char* cmd);
WifiBtErrorCode d_wifi_bt_join_ap(const char* ssid, const char* password);
WifiBtErrorCode d_wifi_bt_rejoin_ap(const char* ssid, const char* password);
WifiBtErrorCode d_wifi_bt_check_ap(void);
WifiBtErrorCode d_wifi_bt_connect(WifiBtConnectInfo* info);
WifiBtErrorCode d_wifi_bt_disconnect(WifiBtConnectInfo info);
WifiBtErrorCode d_wifi_bt_enter_transparent(uint16_t socket_id);
void d_wifi_bt_exit_transparent(void);
WifiBtErrorCode d_wifi_bt_reset(WifiBtWorkMode mode);

WifiBtErrorCode d_wifi_bt_heartbeat(WifiBtConnectInfo* info, ms_t timeout_ms);
WifiBtErrorCode d_wifi_bt_process(uint8_t** const frame_buf, uint16_t* const frame_len);
WifiBtErrorCode d_wifi_bt_send_frame(WifiBtConnectInfo info, const uint8_t* frame, uint16_t length);
WifiBtErrorCode d_wifi_bt_send(WifiBtConnectInfo info, const char* data, uint16_t length);

#endif
