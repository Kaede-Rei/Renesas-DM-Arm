#ifndef _wifi_bt_h_
#define _wifi_bt_h_

#include "infra/delay.h"
#include "platform/uart.h"

#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief WiFi/BT 模块实例 - 用户自定义名称，包含状态码和函数指针
 */
#define wifi wifi_bt_instance

/**
 * @brief WiFi/BT 模块状态码表，使用 X-Macro 定义，方便维护和扩展
 * @param OK 成功
 * @param ERROR 错误
 * @param WAIT_AP 等待连接 AP
 * @param TIMEOUT 超时
 * @param PROCESSING 处理中
 * @param FRAME_READY 有帧可处理
 * @param NO_FRAME 没有帧可处理
 */
#define WIFI_BT_STATUS_TABLE \
    X(OK, 0) \
    X(ERROR, 1) \
    X(WAIT_AP, 2) \
    X(TIMEOUT, 3) \
    X(PROCESSING, 4) \
    X(FRAME_READY, 5) \
    X(NO_FRAME, 6)

/**
 * @brief WiFi/BT 模块状态码，由 X-Macro 自动生成枚举类型
 */
#define X(name, value) WIFI_BT_##name = value,
typedef enum {
    WIFI_BT_STATUS_TABLE
} WifiBtStatus;
#undef X

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

/**
 * @brief WiFi/BT 模块单例结构体，包含状态码和函数指针
 */
#define X(name, value) const WifiBtStatus name;
extern const struct WifiBtInstance {
    /**
     * @brief WiFi/BT 模块状态码
     * @param OK 成功
     * @param ERROR 错误
     * @param WAIT_AP 等待连接 AP
     * @param TIMEOUT 超时
     * @param PROCESSING 处理中
     * @param FRAME_READY 有帧可处理
     * @param NO_FRAME 没有帧可处理
     */
    struct {
        WIFI_BT_STATUS_TABLE
    };
    /**
     * @brief WiFi/BT 模块初始化函数，配置 UART 端口、工作模式、帧解析器等
     * @param uart_instance UART 端口枚举值，表示使用哪个 UART 进行通信
     * @param mode WiFi/BT 工作模式枚举值，表示模块的工作模式（如 STA、SOFT_AP 等）
     * @param header 帧头标识指针，用于帧头匹配
     * @param header_len 帧头标识的长度，最小为 2 字节
     * @return WifiBtStatus 枚举类型，表示操作结果
     */
    WifiBtStatus(*init)(Uart_te uart, WifiBtWorkMode mode, const uint8_t* const header, const uint8_t header_len);
    /**
     * @brief 发送 AT 命令到 WiFi/BT 模块
     * @param cmd 要发送的 AT 命令字符串
     * @return WifiBtStatus 枚举类型，表示操作结果
     */
    WifiBtStatus(*send_cmd)(const char* cmd);
    /**
     * @brief 加入 WiFi AP，连接到指定的 WiFi 网络
     * @param ssid WiFi 网络的 SSID
     * @param pwd WiFi 网络的密码
     * @return WifiBtStatus 枚举类型，表示操作结果
     */
    WifiBtStatus(*join_ap)(const char* ssid, const char* password);
    /**
     * @brief 重新加入 WiFi AP
     * @param ssid WiFi 网络的 SSID
     * @param pwd WiFi 网络的密码
     * @return WifiBtStatus 枚举类型，表示操作结果
     */
    WifiBtStatus(*rejoin_ap)(const char* ssid, const char* password);
    /**
     * @brief 检查 WiFi AP 连接状态
     * @return WifiBtStatus 枚举类型，表示操作结果
     */
    WifiBtStatus(*check_ap)(void);
    /**
     * @brief WiFi/BT 模块连接函数，建立 socket 连接并进入透传模式
     * @param info 包含连接信息的 WifiBtConnectInfo 结构体，至少需要包含 ssid、password、protocol、role、ip、remote_port 和 local_port 字段
     * @return WifiBtStatus 枚举类型，表示操作结果
     */
    WifiBtStatus(*connect)(WifiBtConnectInfo* info);
    /**
     * @brief 断开 WiFi/BT 模块的连接，关闭指定的 socket 连接
     * @param info 包含连接信息的 WifiBtConnectInfo 结构体，至少需要包含 socket_port 字段
     * @return WifiBtStatus 枚举类型，表示操作结果
     */
    WifiBtStatus(*disconnect)(WifiBtConnectInfo* info);
    /**
     * @brief 复位 W800 模块并重新初始化（AT+Z）
     * @param mode 工作模式
     */
    WifiBtStatus(*reset)(WifiBtWorkMode mode);
    /**
     * @brief 进入透传模式
     * @param socket_id 要进入透传模式的 socket 连接 ID，通常由 wifi_bt_connect 函数返回
     * @return WifiBtStatus 枚举类型，表示操作结果
     */
    WifiBtStatus(*enter_transparent)(uint16_t socket_id);
    /**
     * @brief 退出透传模式
     */
    void(*exit_transparent)(void);
    /**
     * @brief 心跳检测函数，定期发送心跳帧并检查连接状态，必要时尝试重连
     * @param info 包含连接信息的 WifiBtConnectInfo 结构体，至少需要包含 socket_port 字段
     * @param timeout_ms 心跳超时时间，单位为毫秒
     * @return WifiBtStatus 枚举类型，表示操作结果
     */
    WifiBtStatus(*heartbeat)(WifiBtConnectInfo* info, ms_t timeout_ms);
    /**
     * @brief 处理 WiFi/BT 模块接收的数据，解析帧数据并检查是否有新的帧就绪
     * @param frame_buf 输出参数，指向存储帧数据的缓冲区指针
     * @param frame_len 输出参数，指向存储帧数据长度的变量指针
     * @return WifiBtStatus 枚举类型，表示操作结果
     */
    WifiBtStatus(*process)(uint8_t** const frame_buf, uint16_t* const frame_len);
    /**
     * @brief 发送数据帧到 WiFi/BT 模块，数据将通过指定的 socket 连接发送
     * @param info 包含连接信息的 WifiBtConnectInfo 结构体，至少需要包含 socket_port 字段
     * @param frame 要发送的数据帧缓冲区指针
     * @param length 要发送的数据帧长度
     * @return WifiBtStatus 枚举类型，表示操作结果
     */
    WifiBtStatus(*send_frame)(WifiBtConnectInfo info, const uint8_t* frame, uint16_t length);
    /**
     * @brief 发送字符串数据，自动包装帧头和长度信息
     * @param info 包含连接信息的 WifiBtConnectInfo 结构体，至少需要包含 socket_port 字段
     * @param data 要发送的字符串数据指针
     * @param length 要发送的字符串数据长度
     * @return WifiBtStatus 枚举类型，表示操作结果
     */
    WifiBtStatus(*send)(WifiBtConnectInfo info, const char* data, uint16_t length);
} wifi_bt_instance;
#undef X

// ! ========================= 接 口 函 数 声 明 ========================= ! //

WifiBtStatus wifi_bt_init(Uart_te uart, WifiBtWorkMode mode, const uint8_t* const header, const uint8_t header_len);
WifiBtStatus wifi_bt_send_cmd(const char* cmd);
WifiBtStatus wifi_bt_join_ap(const char* ssid, const char* password);
WifiBtStatus wifi_bt_rejoin_ap(const char* ssid, const char* password);
WifiBtStatus wifi_bt_check_ap(void);
WifiBtStatus wifi_bt_connect(WifiBtConnectInfo* info);
WifiBtStatus wifi_bt_disconnect(WifiBtConnectInfo* info);
WifiBtStatus wifi_bt_reset(WifiBtWorkMode mode);
WifiBtStatus wifi_bt_enter_transparent(uint16_t socket_id);
void wifi_bt_exit_transparent(void);

WifiBtStatus wifi_bt_heartbeat(WifiBtConnectInfo* info, ms_t timeout_ms);
WifiBtStatus wifi_bt_process(uint8_t** const frame_buf, uint16_t* const frame_len);
WifiBtStatus wifi_bt_send_frame(WifiBtConnectInfo info, const uint8_t* frame, uint16_t length);
WifiBtStatus wifi_bt_send(WifiBtConnectInfo info, const char* data, uint16_t length);

#endif
