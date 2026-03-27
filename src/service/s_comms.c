#include "s_comms.h"

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief WEED 包数据实例
 * @note 该变量用于存储最新的 WEED 包数据，由 process() 函数更新，get_weed() 函数读取
 */
static WeedData weed_comms = { 0 };

/**
 * @brief 通信单例实例
 */
const struct CommsInterface s_comms_instance = {
    {
        #define SX(name, value) .name = value,
        COMMS_STATUS_TABLE
        #undef SX
    },
    .process = s_comms_process,
    .get_weed = s_comms_get_weed,
    .reset = s_comms_reset
};
#define comms s_comms_instance

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static CommsStatus process_weed(const uint8_t* packet, size_t length);

static bool parse_u8(const uint8_t* str, uint8_t* out);
static bool parse_float(const char* str, float* out);
static void rm_str_end_r_n(char* str);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 处理数据包
 * @param packet 数据包指针
 * @param length 数据包长度
 * @return CommsStatus 状态码
 */
CommsStatus s_comms_process(const uint8_t* packet, size_t length) {
    if(packet == NULL || length == 0) return comms.INVALID_PACKET;

    if(length >= 5 && (memcmp((const char*)packet, "WEED:", 5) == 0)) return process_weed(packet, length);

    return comms.INVALID_PACKET;
}

/**
 * @brief 获取识别数据
 * @param data 输出的 WeedData 指针
 * @return CommsStatus 状态码
 */
CommsStatus s_comms_get_weed(WeedData* data) {
    if(data == NULL) return comms.INVALID_ARG;

    *data = weed_comms;
    return comms.OK;
}

/**
 * @brief 重置数据
 * @return CommsStatus 状态码
 */
CommsStatus s_comms_reset(void) {
    weed_comms = (WeedData){ 0 };
    return comms.OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 处理 WEED 包数据
 * @param packet 数据包指针
 * @param length 数据包长度
 * @return CommsStatus 状态码
 */
static CommsStatus process_weed(const uint8_t* packet, size_t length) {
    char temp_buf[length + 1];

    char* payload = NULL;
    char* comma1 = NULL;
    char* comma2 = NULL;
    char* comma3 = NULL;
    char* comma4 = NULL;

    char* id_str = NULL;
    char* x_str = NULL;
    char* y_str = NULL;
    char* z_str = NULL;
    char* confidence_str = NULL;

    WeedData parsed = { 0 };

    if(packet == NULL) return comms.INVALID_ARG;
    if(length == 0) return comms.INVALID_PACKET;

    memcpy(temp_buf, packet, length);
    temp_buf[length] = '\0';

    rm_str_end_r_n(temp_buf);

    if(strncmp(temp_buf, "WEED:", 5) != 0) return comms.INVALID_PACKET;
    payload = temp_buf + 5;
    if(*payload == '\0') return comms.INVALID_PACKET;

    comma1 = strchr(payload, ',');
    if(comma1 == NULL) return comms.INVALID_PACKET;
    comma2 = strchr(comma1 + 1, ',');
    if(comma2 == NULL) return comms.INVALID_PACKET;
    comma3 = strchr(comma2 + 1, ',');
    if(comma3 == NULL) return comms.INVALID_PACKET;
    comma4 = strchr(comma3 + 1, ',');
    if(comma4 == NULL) return comms.INVALID_PACKET;
    if(strchr(comma4 + 1, ',') != NULL) return comms.INVALID_PACKET;

    *comma1 = '\0';
    *comma2 = '\0';
    *comma3 = '\0';
    *comma4 = '\0';

    id_str = payload;
    x_str = comma1 + 1;
    y_str = comma2 + 1;
    z_str = comma3 + 1;
    confidence_str = comma4 + 1;

    if(*id_str == '\0' || *x_str == '\0' || *y_str == '\0' || *z_str == '\0' || *confidence_str == '\0') return comms.INVALID_PACKET;

    if(!parse_u8((uint8_t*)id_str, &parsed.id)) return comms.OUT_OF_RANGE;
    if(!parse_float(x_str, &parsed.x)) return comms.OUT_OF_RANGE;
    if(!parse_float(y_str, &parsed.y)) return comms.OUT_OF_RANGE;
    if(!parse_float(z_str, &parsed.z)) return comms.OUT_OF_RANGE;
    if(!parse_float(confidence_str, &parsed.confidence)) return comms.OUT_OF_RANGE;
    if(parsed.confidence < 0.0f || parsed.confidence > 1.0f) return comms.OUT_OF_RANGE;

    weed_comms = parsed;
    return comms.OK;
}


/**
 * @brief 解析 uint8_t 数值
 * @param str 输入字符串
 * @param out 输出数值指针
 * @return true 成功，false 失败
 */
static bool parse_u8(const uint8_t* str, uint8_t* out) {
    if(str == NULL || out == NULL || *str == '\0') return false;

    char* end_ptr;
    unsigned long value = 0;

    errno = 0;
    value = strtoul((const char*)str, &end_ptr, 10);

    if(errno != 0) return false;
    if(end_ptr == (const char*)str) return false;
    if(*end_ptr != '\0') return false;
    if(value > 255) return false;

    *out = (uint8_t)value;
    return true;
}

/**
 * @brief 解析浮点数数值
 * @param str 输入字符串
 * @param out 输出数值指针
 * @return true 成功，false 失败
 */
static bool parse_float(const char* str, float* out) {
    if(str == NULL || out == NULL || *str == '\0') return false;

    char* end_ptr;
    float value = 0.0f;

    errno = 0;
    value = strtof(str, &end_ptr);

    if(errno != 0) return false;
    if(end_ptr == str) return false;
    if(*end_ptr != '\0') return false;

    *out = value;
    return true;
}

/**
 * @brief 移除字符串末尾的 \r 和 \n 字符
 * @param str 输入字符串
 */
static void rm_str_end_r_n(char* str) {
    if(str == NULL) return;

    size_t len = strlen(str);
    while(len > 0 && (str[len - 1] == '\r' || str[len - 1] == '\n')) {
        str[len - 1] = '\0';
        --len;
    }
}

#undef comms
