#include "d_wifi_bt.h"

#include "drive/d_systick.h"
#include "tools/ring_buf.h"
#include "tools/simple_api.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// ! ========================= 变 量 声 明 ========================= ! //

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
    ms_t last_recv_time;
    const uint8_t* header;
    uint8_t header_len;

    uint8_t rx_raw[WIFI_BT_FRAME_RX_BUF_SIZE];
    RingBuf rx_buf;

    uint8_t frame_raw[WIFI_BT_FRAME_BUF_SIZE];
    FrameParser frame_parser;

} wifi_bt_config = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void wifi_bt_send_raw(const char* str);
static bool wifi_bt_wait_str(const char* expected, uint8_t length, uint32_t timeout_ms);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief WiFi/BT 模块初始化函数，配置 UART 端口、工作模式、帧解析器等
 * @param uart UART 端口枚举值，表示使用哪个 UART 进行通信
 * @param mode WiFi/BT 工作模式枚举值，表示模块的工作模式（如 STA、SOFT_AP 等）
 * @param header 帧头标识指针，用于帧头匹配
 * @param header_len 帧头标识的长度，最小为 2 字节
 * @return WifiBtErrorCode 枚举类型，表示操作结果
 */
WifiBtErrorCode d_wifi_bt_init(Uart_te uart, WifiBtWorkMode mode, const uint8_t* const header, const uint8_t header_len) {
    wifi_bt_config.uart = uart;
    wifi_bt_config.header = header;
    wifi_bt_config.header_len = header_len;

    RingBufCreate(&wifi_bt_config.rx_buf, wifi_bt_config.rx_raw, WIFI_BT_FRAME_RX_BUF_SIZE, 1);
    d_uart_init(uart, &wifi_bt_config.rx_buf);

    FrameParserCreate(&wifi_bt_config.frame_parser, &wifi_bt_config.rx_buf, header, header_len, wifi_bt_config.frame_raw, WIFI_BT_FRAME_BUF_SIZE, false);

    gpio_write(BSP_IO_PORT_05_PIN_08, BSP_IO_LEVEL_LOW);
    s_delay_ms(100);
    gpio_write(BSP_IO_PORT_05_PIN_08, BSP_IO_LEVEL_HIGH);
    s_delay_ms(500);

    char str[12];
    snprintf(str, sizeof(str), "AT+WPRT=%d\r\n", mode);
    wifi_bt_send_raw(str);
    if(!wifi_bt_wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;

    wifi_bt_send_raw("AT+SKRPTM=1\r\n");
    if(!wifi_bt_wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;

    return WIFI_BT_SUCCESS;
}

/**
 * @brief 发送 AT 命令到 WiFi/BT 模块
 * @param cmd 要发送的 AT 命令字符串
 * @return WifiBtErrorCode 枚举类型，表示操作结果
 */
WifiBtErrorCode d_wifi_bt_send_cmd(const char* cmd) {
    char buf[64];
    snprintf(buf, sizeof(buf), "%s\r\n", cmd);

    wifi_bt_send_raw(buf);

    return WIFI_BT_SUCCESS;
}

/**
 * @brief 加入 WiFi AP，连接到指定的 WiFi 网络
 * @param ssid WiFi 网络的 SSID
 * @param pwd WiFi 网络的密码
 * @return WifiBtErrorCode 枚举类型，表示操作结果
 */
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

/**
 * @brief 重新加入 WiFi AP，即断开当前连接并尝试重新连接到指定的 WiFi 网络
 * @param ssid WiFi 网络的 SSID，为 NULL 时表示使用上次连接的 SSID
 * @param pwd WiFi 网络的密码，为 NULL 时表示使用上次连接的密码
 * @return WifiBtErrorCode 枚举类型，表示操作结果
 */
WifiBtErrorCode d_wifi_bt_rejoin_ap(const char* ssid, const char* password) {
    uint8_t dummy;

    while(wifi_bt_config.rx_buf.read(&wifi_bt_config.rx_buf, &dummy) == RING_BUF_SUCCESS);
    wifi_bt_config.frame_parser.reset(&wifi_bt_config.frame_parser);

    if(!(ssid && password)) {
        wifi_bt_send_raw("AT+WJOIN\r\n");
        if(!wifi_bt_wait_str("+OK", 3, 10000)) return WIFI_BT_ERROR;
    }
    else d_wifi_bt_join_ap(ssid, password);

    return WIFI_BT_SUCCESS;
}

/**
 * @brief 查询 WiFi STA 连接状态（AT+LKSTT）
 * @return WIFI_BT_SUCCESS 表示已连接，WIFI_BT_ERROR 表示未连接
 */
WifiBtErrorCode d_wifi_bt_check_ap(void) {
    uint8_t dummy;
    while(wifi_bt_config.rx_buf.read(&wifi_bt_config.rx_buf, &dummy) == RING_BUF_SUCCESS);
    wifi_bt_config.frame_parser.reset(&wifi_bt_config.frame_parser);

    wifi_bt_send_raw("AT+LKSTT\r\n");
    return wifi_bt_wait_str("+OK=1", 5, 1000) ? WIFI_BT_SUCCESS : WIFI_BT_ERROR;
}

/**
 * @brief 处理 WiFi/BT 模块接收的数据，解析帧数据并检查是否有新的帧就绪
 * @param frame_buf 输出参数，指向存储帧数据的缓冲区指针
 * @param frame_len 输出参数，指向存储帧数据长度的变量指针
 * @return WifiBtErrorCode 枚举类型，表示操作结果
 */
WifiBtErrorCode d_wifi_bt_connect(WifiBtConnectInfo* info) {
    char cmd[128];
    uint8_t dummy;

    while(wifi_bt_config.rx_buf.read(&wifi_bt_config.rx_buf, &dummy) == RING_BUF_SUCCESS);
    wifi_bt_config.frame_parser.reset(&wifi_bt_config.frame_parser);

    snprintf(cmd, sizeof(cmd), "AT+SKCT=%d,%d,\"%s\",%d,%d",
        info->protocol, info->role, info->ip, info->remote_port, info->local_port);
    if(d_wifi_bt_send_cmd(cmd) != WIFI_BT_SUCCESS) return WIFI_BT_ERROR;
    if(!wifi_bt_wait_str("+OK=", 4, 3000)) return WIFI_BT_ERROR;

    char buf[32] = { 0 };
    uint8_t idx = 0;
    char ch;
    ms_t start_time = d_systick_get_ms();
    while(d_systick_get_ms() - start_time < 1000) {
        if(wifi_bt_config.rx_buf.read(&wifi_bt_config.rx_buf, (uint8_t*)&ch) == RING_BUF_SUCCESS) {
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

    return WIFI_BT_SUCCESS;
}

/**
 * @brief 断开 WiFi/BT 模块的连接，关闭指定的 socket 连接
 * @param info 包含连接信息的 WifiBtConnectInfo 结构体，至少需要包含 socket_port 字段
 * @return WifiBtErrorCode 枚举类型，表示操作结果
 */
WifiBtErrorCode d_wifi_bt_disconnect(WifiBtConnectInfo info) {
    char cmd[128];

    snprintf(cmd, sizeof(cmd), "AT+SKCLS=%d", info.socket_port);
    if(d_wifi_bt_send_cmd(cmd) != WIFI_BT_SUCCESS) return WIFI_BT_ERROR;
    if(!wifi_bt_wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;

    return WIFI_BT_SUCCESS;
}

/**
 * @brief 复位 W800 模块并重新初始化（AT+Z）
 * @param mode 工作模式
 */
WifiBtErrorCode d_wifi_bt_reset(WifiBtWorkMode mode) {
    uint8_t dummy;

    gpio_write(BSP_IO_PORT_05_PIN_08, BSP_IO_LEVEL_LOW);
    s_delay_ms(100);
    gpio_write(BSP_IO_PORT_05_PIN_08, BSP_IO_LEVEL_HIGH);
    s_delay_ms(500);

    while(wifi_bt_config.rx_buf.read(&wifi_bt_config.rx_buf, &dummy) == RING_BUF_SUCCESS);
    wifi_bt_config.frame_parser.reset(&wifi_bt_config.frame_parser);

    char str[12];
    snprintf(str, sizeof(str), "AT+WPRT=%d\r\n", mode);
    wifi_bt_send_raw(str);
    if(!wifi_bt_wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;

    wifi_bt_send_raw("AT+SKRPTM=1\r\n");
    if(!wifi_bt_wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;

    return WIFI_BT_SUCCESS;
}

WifiBtErrorCode d_wifi_bt_heartbeat(WifiBtConnectInfo* info, ms_t timeout_ms) {
    static bool is_connected = false;
    static uint8_t retry_count = 0;
    static ms_t start_time = 0;

    if(s_nb_delay_ms(&start_time, timeout_ms)) {
        if(!is_connected) {
            if(d_wifi_bt_check_ap() != WIFI_BT_SUCCESS) {
                printf("WiFi 未连接，尝试重连...\r\n");
                if(d_wifi_bt_rejoin_ap(NULL, NULL) != WIFI_BT_SUCCESS)
                    printf("WiFi 重连失败，继续等待...\r\n");
                return WIFI_BT_WAIT_AP;
            }

            printf("尝试链接...(retry_count=%d)\r\n", retry_count);
            if(d_wifi_bt_connect(info) == WIFI_BT_SUCCESS) {
                is_connected = true;
                retry_count = 0;
                wifi_bt_config.last_recv_time = d_systick_get_ms();
                printf("连接成功!\r\n");
            }
            else {
                if(++retry_count >= 3) {
                    printf("连接失败，重置模块...\r\n");
                    retry_count = 0;
                    if(d_wifi_bt_reset(STA) == WIFI_BT_SUCCESS)
                        d_wifi_bt_join_ap(info->ssid, info->password);
                }
            }
        }

        if(is_connected) {
            uint8_t heart_frame[wifi_bt_config.header_len + 2 + 5];
            memcpy(heart_frame, wifi_bt_config.header, wifi_bt_config.header_len);
            heart_frame[wifi_bt_config.header_len] = 0x00;
            heart_frame[wifi_bt_config.header_len + 1] = 0x05;
            memcpy(heart_frame + wifi_bt_config.header_len + 2, "HEART", 5);
            bool send_ok = (d_wifi_bt_send_frame(*info, heart_frame, (uint16_t)sizeof(heart_frame)) == WIFI_BT_SUCCESS);

            if(info->protocol == TCP) {
                if(!send_ok) {
                    printf("发送失败，触发重连\r\n");
                    d_wifi_bt_disconnect(*info);
                    is_connected = false;
                }
            }
            else {
                if(!send_ok || d_systick_get_ms() - wifi_bt_config.last_recv_time > timeout_ms * 3) {
                    printf("UDP 超时，触发重连\r\n");
                    d_wifi_bt_disconnect(*info);
                    is_connected = false;
                    retry_count = 0;
                }
            }
        }
    }

    return is_connected ? WIFI_BT_SUCCESS : WIFI_BT_ERROR;
}

/**
 * @brief 处理 WiFi/BT 模块接收的数据，解析帧数据并检查是否有新的帧就绪
 * @param frame_buf 输出参数，指向存储帧数据的缓冲区指针
 * @param frame_len 输出参数，指向存储帧数据长度的变量指针
 * @return WifiBtErrorCode 枚举类型，表示操作结果
 */
WifiBtErrorCode d_wifi_bt_process(uint8_t** const frame_buf, uint16_t* const frame_len) {
    if(wifi_bt_config.frame_parser.process(&wifi_bt_config.frame_parser) == FRAME_PARSER_PROCESSING) return WIFI_BT_PROCESSING;
    if(wifi_bt_config.frame_parser.get_frame(&wifi_bt_config.frame_parser, frame_buf, frame_len) == FRAME_PARSER_SUCCESS) {
        wifi_bt_config.last_recv_time = d_systick_get_ms();
        if(*frame_len == 5 && memcmp(*frame_buf, "ALIVE", 5) == 0) {
            wifi_bt_config.frame_parser.finish(&wifi_bt_config.frame_parser);
            return WIFI_BT_NO_FRAME;
        }

        return WIFI_BT_FRAME_READY;
    }

    return WIFI_BT_NO_FRAME;
}

/**
 * @brief 发送数据帧到 WiFi/BT 模块，数据将通过指定的 socket 连接发送
 * @param info 包含连接信息的 WifiBtConnectInfo 结构体，至少需要包含 socket_port 字段
 * @param data 要发送的数据缓冲区指针
 * @param length 要发送的数据长度
 * @return WifiBtErrorCode 枚举类型，表示操作结果
 */
WifiBtErrorCode d_wifi_bt_send_frame(WifiBtConnectInfo info, const uint8_t* data, uint16_t length) {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+SKSND=%d,%d", info.socket_port, length);
    if(d_wifi_bt_send_cmd(cmd) != WIFI_BT_SUCCESS) return WIFI_BT_ERROR;
    if(!wifi_bt_wait_str("+OK", 3, 1000)) return WIFI_BT_ERROR;
    wifi_bt_wait_str("\r\n\r\n", 4, 200);

    d_uart_write(wifi_bt_config.uart, data, length);
    d_uart_wait_tx_complete(wifi_bt_config.uart);

    return WIFI_BT_SUCCESS;
}

/**
 * @brief 标记当前帧已处理完毕，将状态机复位至空闲
 */
void d_wifi_bt_finish_frame(void) {
    wifi_bt_config.frame_parser.finish(&wifi_bt_config.frame_parser);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 向 WiFi/BT 模块发送原始字符串数据，通常用于发送 AT 命令
 * @param str 要发送的字符串数据
 */
static void wifi_bt_send_raw(const char* str) {
    d_uart_write(wifi_bt_config.uart, (uint8_t*)str, (uint16_t)strlen(str));
    d_uart_wait_tx_complete(wifi_bt_config.uart);
}

/**
 * @brief 等待 WiFi/BT 模块返回指定的字符串响应，通常用于等待 AT 命令的响应
 * @param expected 期望接收的字符串响应
 * @param length 期望字符串的长度
 * @param timeout_ms 等待响应的超时时间，单位为毫秒
 * @return true 表示在超时时间内成功接收到期望的字符串响应，false 表示超时未接收到或接收到其他数据
 */
static bool wifi_bt_wait_str(const char* expected, uint8_t length, uint32_t timeout_ms) {
    int idx = 0;
    char ch;
    ms_t start_time = d_systick_get_ms();

    while(d_systick_get_ms() - start_time < timeout_ms) {
        if(wifi_bt_config.rx_buf.read(&wifi_bt_config.rx_buf, (uint8_t*)&ch) == RING_BUF_SUCCESS) {
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