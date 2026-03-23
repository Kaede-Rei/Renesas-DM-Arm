#ifndef _s_comms_h_
#define _s_comms_h_

#include <stdint.h>
#include <stddef.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 通信单例用户自定义名称
 */
#define comms s_comms_instance

/**
 * @brief 通信错误码
 * @param COMMS_SUCCESS 操作成功
 * @param COMMS_ERROR_INVALID_PACKET 数据包无效
 */
typedef enum {
    COMMS_SUCCESS = 0,
    COMMS_ERROR_INVALID_PACKET,
} CommsErrorCode;

typedef struct {
    uint8_t id;
    uint16_t x;
    uint16_t y;
    float confidence;
} WeedData;

/**
 * @brief 通信单例接口
 * @param process_packet 处理数据包的函数指针
 * @param get_weed_data 获取识别数据的函数指针
 */
extern const struct CommsInterface {
    /**
     * @brief 处理数据包
     * @param packet 数据包指针
     * @param length 数据包长度
     * @return CommsErrorCode 错误码
     */
    CommsErrorCode(*process_packet)(const uint8_t* packet, size_t length);

    /**
     * @brief 获取识别数据
     * @param data 输出的 WeedData 指针
     * @return CommsErrorCode 错误码
     */
    CommsErrorCode(*get_weed_data)(WeedData* data);
} s_comms_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

CommsErrorCode s_comms_process_packet(const uint8_t* packet, size_t length);
CommsErrorCode s_comms_get_weed_data(WeedData* data);

#endif
