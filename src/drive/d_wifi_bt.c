#include "d_wifi_bt.h"

#include "drive/d_systick.h"
#include "drive/d_uart.h"
#include "service/s_delay.h"
#include "tools/protocol_parser.h"
#include "tools/simple_api.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// ! ========================= 变 量 声 明 ========================= ! //

const struct WifiBtInstance d_wifi_bt_instance = {
    {
        #define X(name, value) .name = WIFI_BT_##name,
        WIFI_BT_STATUS_TABLE
        #undef X
    },

    .init = d_wifi_bt_init,
    .send_cmd = d_wifi_bt_send_cmd,
    .join_ap = d_wifi_bt_join_ap,
    .rejoin_ap = d_wifi_bt_rejoin_ap,
    .check_ap = d_wifi_bt_check_ap,
    .connect = d_wifi_bt_connect,
    .disconnect = d_wifi_bt_disconnect,
    .reset = d_wifi_bt_reset,
    .enter_transparent = d_wifi_bt_enter_transparent,
    .exit_transparent = d_wifi_bt_exit_transparent,

    .heartbeat = d_wifi_bt_heartbeat,
    .process = d_wifi_bt_process,
    .send_frame = d_wifi_bt_send_frame,
    .send = d_wifi_bt_send
};

/**
 * @brief WiFi/BT 模块配置结构体，包含 UART 端口、帧接收缓冲区、帧解析器等相关配置
 * @param uart UART 端口枚举值，表示使用哪个 UART 进行通信
 * @param last_recv_time 上次接收数据的时间戳，用于心跳检测和超时处理
 * @param frame_ready 帧就绪标志，表示是否有新的帧数据可供处理
 * @param rx_raw 原始接收数据缓冲区，用于存储从 UART 接收的原始数据
 * @param rx_buf 环形缓冲区结构体，用于管理接收数据的存储和读取
 * @param frame_raw 帧数据缓冲区，用于存储解析完成的帧数据
 * @param frame_parser 帧解析器结构体，用于解析接收的原始数据
 */
static struct {
    Uart_te uart;
    WifiBtWorkMode mode;
    ms_t last_recv_time;
    bool is_transparent_mode;

    const uint8_t* header;
    uint8_t header_len;

    uint8_t rx_raw[WIFI_BT_FRAME_RX_BUF_SIZE];
    RingBuf rx_buf;

    uint8_t frame_raw[WIFI_BT_FRAME_BUF_SIZE];
    FrameParser frame_parser;

} config = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void send_raw(const char* str);
static bool wait_str(const char* expected, uint8_t length, uint32_t timeout_ms);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief WiFi/BT 模块初始化函数，配置 UART 端口、工作模式、帧解析器等
 * @param uart_instance UART 端口枚举值，表示使用哪个 UART 进行通信
 * @param mode WiFi/BT 工作模式枚举值，表示模块的工作模式（如 STA、SOFT_AP 等）
 * @param header 帧头标识指针，用于帧头匹配
 * @param header_len 帧头标识的长度，最小为 2 字节
 * @return WifiBtStatus 枚举类型，表示操作结果
 */
WifiBtStatus d_wifi_bt_init(Uart_te uart_instance, WifiBtWorkMode mode, const uint8_t* const header, const uint8_t header_len) {
    config.uart = uart_instance;
    config.mode = mode;
    config.header = header;
    config.header_len = header_len;

    RingBufCreate(&config.rx_buf, config.rx_raw, WIFI_BT_FRAME_RX_BUF_SIZE, 1);
    d_uart_init(uart_instance, &config.rx_buf);

    FrameParserCreate(&config.frame_parser, &config.rx_buf, header, header_len, config.frame_raw, WIFI_BT_FRAME_BUF_SIZE, false);

    gpio_write(BSP_IO_PORT_05_PIN_08, BSP_IO_LEVEL_LOW);
    s_delay_ms(100);
    gpio_write(BSP_IO_PORT_05_PIN_08, BSP_IO_LEVEL_HIGH);
    s_delay_ms(500);

    char str[12];
    snprintf(str, sizeof(str), "AT+WPRT=%d\r\n", mode);
    send_raw(str);
    if(!wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;

    send_raw("AT+SKRPTM=1\r\n");
    if(!wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;

    return WIFI_BT_OK;
}

/**
 * @brief 发送 AT 命令到 WiFi/BT 模块
 * @param cmd 要发送的 AT 命令字符串
 * @return WifiBtStatus 枚举类型，表示操作结果
 */
WifiBtStatus d_wifi_bt_send_cmd(const char* cmd) {
    char buf[64];
    snprintf(buf, sizeof(buf), "%s\r\n", cmd);

    send_raw(buf);

    return WIFI_BT_OK;
}

/**
 * @brief 加入 WiFi AP，连接到指定的 WiFi 网络
 * @param ssid WiFi 网络的 SSID
 * @param pwd WiFi 网络的密码
 * @return WifiBtStatus 枚举类型，表示操作结果
 */
WifiBtStatus d_wifi_bt_join_ap(const char* ssid, const char* pwd) {
    char cmd[128];

    snprintf(cmd, sizeof(cmd), "AT+SSID=%s", ssid);
    if(d_wifi_bt_send_cmd(cmd) != WIFI_BT_OK) return WIFI_BT_ERROR;
    if(!wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;

    snprintf(cmd, sizeof(cmd), "AT+KEY=1,0,%s", pwd);
    if(d_wifi_bt_send_cmd(cmd) != WIFI_BT_OK) return WIFI_BT_ERROR;
    if(!wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;

    snprintf(cmd, sizeof(cmd), "AT+WJOIN");
    if(d_wifi_bt_send_cmd(cmd) != WIFI_BT_OK) return WIFI_BT_ERROR;
    if(!wait_str("+OK", 3, 10000)) return WIFI_BT_ERROR;

    return WIFI_BT_OK;
}

/**
 * @brief 重新加入 WiFi AP，即断开当前连接并尝试重新连接到指定的 WiFi 网络
 * @param ssid WiFi 网络的 SSID，为 NULL 时表示使用上次连接的 SSID
 * @param pwd WiFi 网络的密码，为 NULL 时表示使用上次连接的密码
 * @return WifiBtStatus 枚举类型，表示操作结果
 */
WifiBtStatus d_wifi_bt_rejoin_ap(const char* ssid, const char* password) {
    uint8_t dummy;

    while(config.rx_buf.read(&config.rx_buf, &dummy) == RING_BUF_SUCCESS);
    config.frame_parser.reset(&config.frame_parser);

    if(!(ssid && password)) {
        send_raw("AT+WJOIN\r\n");
        if(!wait_str("+OK", 3, 10000)) return WIFI_BT_ERROR;
    }
    else d_wifi_bt_join_ap(ssid, password);

    return WIFI_BT_OK;
}

/**
 * @brief 查询 WiFi STA 连接状态（AT+LKSTT）
 * @return WIFI_BT_OK 表示已连接，WIFI_BT_ERROR 表示未连接
 */
WifiBtStatus d_wifi_bt_check_ap(void) {
    uint8_t dummy;
    while(config.rx_buf.read(&config.rx_buf, &dummy) == RING_BUF_SUCCESS);
    config.frame_parser.reset(&config.frame_parser);

    send_raw("AT+LKSTT\r\n");
    return wait_str("+OK=1", 5, 1000) ? WIFI_BT_OK : WIFI_BT_ERROR;
}

/**
 * @brief WiFi/BT 模块连接函数，建立 socket 连接并进入透传模式
 * @param info 包含连接信息的 WifiBtConnectInfo 结构体，至少需要包含 ssid、password、protocol、role、ip、remote_port 和 local_port 字段
 * @return WifiBtStatus 枚举类型，表示操作结果
 */
WifiBtStatus d_wifi_bt_connect(WifiBtConnectInfo* info) {
    char cmd[128];
    uint8_t dummy;

    while(config.rx_buf.read(&config.rx_buf, &dummy) == RING_BUF_SUCCESS);
    config.frame_parser.reset(&config.frame_parser);

    snprintf(cmd, sizeof(cmd), "AT+SKCT=%d,%d,\"%s\",%d,%d",
        info->protocol, info->role, info->ip, info->remote_port, info->local_port);
    if(d_wifi_bt_send_cmd(cmd) != WIFI_BT_OK) return WIFI_BT_ERROR;
    if(!wait_str("+OK=", 4, 3000)) return WIFI_BT_ERROR;

    char buf[32] = { 0 };
    uint8_t idx = 0;
    char ch;
    ms_t start_time = d_systick_get_ms();
    while(d_systick_get_ms() - start_time < 1000) {
        if(config.rx_buf.read(&config.rx_buf, (uint8_t*)&ch) == RING_BUF_SUCCESS) {
            if(ch == '\r' || ch == '\n') {
                if(idx > 0)
                    break;
            }
            else if(idx < sizeof(buf) - 1) buf[idx++] = (char)ch;
            else return WIFI_BT_ERROR;
        }
        else s_delay_ms(1);
    }

    info->socket_port = (uint16_t)atoi(buf);
    printf("IP:%s - SocketPort:%d\r\n", info->ip, info->socket_port);

    return WIFI_BT_OK;
}

/**
 * @brief 断开 WiFi/BT 模块的连接，关闭指定的 socket 连接
 * @param info 包含连接信息的 WifiBtConnectInfo 结构体，至少需要包含 socket_port 字段
 * @return WifiBtStatus 枚举类型，表示操作结果
 */
WifiBtStatus d_wifi_bt_disconnect(WifiBtConnectInfo* info) {
    char cmd[128];

    snprintf(cmd, sizeof(cmd), "AT+SKCLS=%d", info->socket_port);
    if(d_wifi_bt_send_cmd(cmd) != WIFI_BT_OK) return WIFI_BT_ERROR;
    if(!wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;

    return WIFI_BT_OK;
}

/**
 * @brief 复位 W800 模块并重新初始化（AT+Z）
 * @param mode 工作模式
 */
WifiBtStatus d_wifi_bt_reset(WifiBtWorkMode mode) {
    uint8_t dummy;

    gpio_write(BSP_IO_PORT_05_PIN_08, BSP_IO_LEVEL_LOW);
    s_delay_ms(100);
    gpio_write(BSP_IO_PORT_05_PIN_08, BSP_IO_LEVEL_HIGH);
    s_delay_ms(500);

    while(config.rx_buf.read(&config.rx_buf, &dummy) == RING_BUF_SUCCESS);
    config.frame_parser.reset(&config.frame_parser);

    char str[12];
    snprintf(str, sizeof(str), "AT+WPRT=%d\r\n", mode);
    send_raw(str);
    if(!wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;

    send_raw("AT+SKRPTM=1\r\n");
    if(!wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;

    return WIFI_BT_OK;
}

/**
 * @brief 进入透传模式
 * @param socket_id 要进入透传模式的 socket 连接 ID，通常由 d_wifi_bt_connect 函数返回
 * @return WifiBtStatus 枚举类型，表示操作结果
 */
WifiBtStatus d_wifi_bt_enter_transparent(uint16_t socket_id) {
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+SKSDF=%d\r\n", socket_id);
    send_raw(cmd);
    if(!wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;

    send_raw("AT+ENTM\r\n");
    if(!wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;

    config.is_transparent_mode = true;

    uint8_t dummy;
    while(config.rx_buf.read(&config.rx_buf, &dummy) == RING_BUF_SUCCESS);
    config.frame_parser.reset(&config.frame_parser);

    config.last_recv_time = d_systick_get_ms();
    return WIFI_BT_OK;
}

/**
 * @brief 退出透传模式
 */
void d_wifi_bt_exit_transparent(void) {
    if(!config.is_transparent_mode) return;

    s_delay_ms(300);
    d_uart_write(config.uart, (uint8_t*)"+++", 3);
    d_uart_wait_tx_complete(config.uart);
    s_delay_ms(300);

    config.is_transparent_mode = false;

    uint8_t dummy;
    while(config.rx_buf.read(&config.rx_buf, &dummy) == RING_BUF_SUCCESS);
}

/**
 * @brief 心跳检测函数，定期发送心跳帧并检查连接状态，必要时尝试重连
 * @param info 包含连接信息的 WifiBtConnectInfo 结构体，至少需要包含 socket_port 字段
 * @param timeout_ms 心跳超时时间，单位为毫秒
 * @return WifiBtStatus 枚举类型，表示操作结果
 */
WifiBtStatus d_wifi_bt_heartbeat(WifiBtConnectInfo* info, ms_t timeout_ms) {
    static bool is_connected = false;
    static ms_t last_heartbeat_tick = 0;
    ms_t now = d_systick_get_ms();
    static uint8_t retry_count = 0;

    if(is_connected && s_nb_delay_ms(&last_heartbeat_tick, timeout_ms)) {
        uint8_t heart_frame[config.header_len + 2 + 5];
        uint16_t header_len = config.header_len;

        memcpy(heart_frame, config.header, header_len);
        heart_frame[header_len] = 0x00;
        heart_frame[header_len + 1] = 0x05;
        memcpy(heart_frame + header_len + 2, "HEART", 5);

        d_wifi_bt_send_frame(*info, heart_frame, (uint16_t)(header_len + 2 + 5));
    }

    if(is_connected && (now - config.last_recv_time > timeout_ms * 5)) {
        printf("心跳超时，触发重连\r\n");
        is_connected = false;
    }

    if(!is_connected) {
        d_wifi_bt_exit_transparent();

        if(d_wifi_bt_check_ap() != WIFI_BT_OK) {
            printf("WiFi 未连接，尝试重连...\r\n");
            if(d_wifi_bt_rejoin_ap(NULL, NULL) != WIFI_BT_OK) {
                printf("WiFi 重连失败，继续等待...\r\n");
                return WIFI_BT_WAIT_AP;
            }
        }

        printf("尝试链接 Socket...(retry_count=%d)\r\n", retry_count);
        if(d_wifi_bt_connect(info) == WIFI_BT_OK) {
            if(d_wifi_bt_enter_transparent(info->socket_port) == WIFI_BT_OK) {
                is_connected = true;
                retry_count = 0;
                config.last_recv_time = d_systick_get_ms();
                printf("连接成功，进入透传模式\r\n");
            }
            else {
                d_wifi_bt_disconnect(info);
                printf("进入透传模式失败，断开连接重试...\r\n");
            }
        }
        else {
            printf("连接失败，重置模块...\r\n");
            retry_count = 0;
            if(d_wifi_bt_reset(config.mode) == WIFI_BT_OK)
                d_wifi_bt_join_ap(info->ssid, info->password);
        }
    }

    return is_connected ? WIFI_BT_OK : WIFI_BT_ERROR;
}

/**
 * @brief 处理 WiFi/BT 模块接收的数据，解析帧数据并检查是否有新的帧就绪
 * @param frame_buf 输出参数，指向存储帧数据的缓冲区指针
 * @param frame_len 输出参数，指向存储帧数据长度的变量指针
 * @return WifiBtStatus 枚举类型，表示操作结果
 */
WifiBtStatus d_wifi_bt_process(uint8_t** const frame_buf, uint16_t* const frame_len) {
    static uint8_t wifi_bt_stable_buf[WIFI_BT_FRAME_BUF_SIZE];
    static uint16_t wifi_bt_stable_len = 0;
    uint8_t* raw_buf;
    uint16_t raw_len;

    if(config.frame_parser.process(&config.frame_parser) == FRAME_PARSER_PROCESSING) return WIFI_BT_PROCESSING;
    if(config.frame_parser.get_frame(&config.frame_parser, &raw_buf, &raw_len) != FRAME_PARSER_SUCCESS) return WIFI_BT_NO_FRAME;
    config.frame_parser.finish(&config.frame_parser);

    if(raw_len == 5 && memcmp(raw_buf, "ALIVE", 5) == 0) {
        config.last_recv_time = d_systick_get_ms();
        *frame_buf = NULL;
        *frame_len = 0;
        return WIFI_BT_NO_FRAME;
    }

    wifi_bt_stable_len = (raw_len < WIFI_BT_FRAME_BUF_SIZE) ? raw_len : WIFI_BT_FRAME_BUF_SIZE;
    memcpy(wifi_bt_stable_buf, raw_buf, wifi_bt_stable_len);
    *frame_buf = wifi_bt_stable_buf;
    *frame_len = wifi_bt_stable_len;

    return WIFI_BT_FRAME_READY;
}

/**
 * @brief 发送数据帧到 WiFi/BT 模块，数据将通过指定的 socket 连接发送
 * @param info 包含连接信息的 WifiBtConnectInfo 结构体，至少需要包含 socket_port 字段
 * @param frame 要发送的数据帧缓冲区指针
 * @param length 要发送的数据帧长度
 * @return WifiBtStatus 枚举类型，表示操作结果
 */
WifiBtStatus d_wifi_bt_send_frame(WifiBtConnectInfo info, const uint8_t* frame, uint16_t length) {
    if(config.is_transparent_mode) {
        d_uart_write(config.uart, frame, length);
        d_uart_wait_tx_complete(config.uart);

        return WIFI_BT_OK;
    }
    else {
        char cmd[64];
        snprintf(cmd, sizeof(cmd), "AT+SKSND=%d,%d", info.socket_port, length);
        if(d_wifi_bt_send_cmd(cmd) != WIFI_BT_OK) return WIFI_BT_ERROR;
        if(!wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;

        uint8_t dummy;
        ms_t last_byte_time = d_systick_get_ms();
        while(d_systick_get_ms() - last_byte_time < 50) {
            if(config.rx_buf.read(&config.rx_buf, &dummy) == RING_BUF_SUCCESS)
                last_byte_time = d_systick_get_ms();
        }

        d_uart_write(config.uart, frame, length);
        d_uart_wait_tx_complete(config.uart);

        return WIFI_BT_OK;
    }
}

/**
 * @brief 发送字符串数据，自动包装帧头和长度信息
 * @param info 包含连接信息的 WifiBtConnectInfo 结构体，至少需要包含 socket_port 字段
 * @param data 要发送的字符串数据指针
 * @param length 要发送的字符串数据长度
 * @return WifiBtStatus 枚举类型，表示操作结果
 */
WifiBtStatus d_wifi_bt_send(WifiBtConnectInfo info, const char* data, uint16_t length) {
    uint8_t frame[config.header_len + 2 + length];
    uint16_t header_len = config.header_len;

    memcpy(frame, config.header, header_len);
    frame[header_len] = (uint8_t)((length >> 8) & 0xFF);
    frame[header_len + 1] = (uint8_t)(length & 0xFF);
    memcpy(frame + header_len + 2, data, length);

    uint8_t print_buf[(uint16_t)(header_len + 2 + length) + 1];
    uint16_t copy_len = (uint16_t)(header_len + 2 + length);
    memcpy(print_buf, frame, copy_len);

    return d_wifi_bt_send_frame(info, (const uint8_t*)frame, (uint16_t)(header_len + 2 + length));
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 向 WiFi/BT 模块发送原始字符串数据，通常用于发送 AT 命令
 * @param str 要发送的字符串数据
 */
static void send_raw(const char* str) {
    d_uart_write(config.uart, (uint8_t*)str, (uint16_t)strlen(str));
    d_uart_wait_tx_complete(config.uart);
}

/**
 * @brief 等待 WiFi/BT 模块返回指定的字符串响应，通常用于等待 AT 命令的响应
 * @param expected 期望接收的字符串响应
 * @param length 期望字符串的长度
 * @param timeout_ms 等待响应的超时时间，单位为毫秒
 * @return true 表示在超时时间内成功接收到期望的字符串响应，false 表示超时未接收到或接收到其他数据
 */
static bool wait_str(const char* expected, uint8_t length, uint32_t timeout_ms) {
    int idx = 0;
    char ch;
    ms_t start_time = d_systick_get_ms();

    while(d_systick_get_ms() - start_time < timeout_ms) {
        if(config.rx_buf.read(&config.rx_buf, (uint8_t*)&ch) == RING_BUF_SUCCESS) {
            if(ch == expected[idx]) {
                if(++idx == length)
                    return true;
            }
            else idx = (ch == expected[0]) ? 1 : 0;
        }
        else s_delay_ms(1);
    }

    return false;
}
