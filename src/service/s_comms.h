#ifndef _s_comms_h_
#define _s_comms_h_

#include <stdint.h>
#include <stddef.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 通信单例，由用户自定义名称
 */
#define comms s_comms_instance

/**
 * @brief 通信状态码表
 * @param OK 操作成功
 * @param INVALID_PACKET 数据包无效
 */
#define COMMS_STATUS_TABLE \
    SX(OK, 0) \
    SX(INVALID_PACKET, 1) \
    SX(INVALID_ARG, 2) \
    SX(OUT_OF_RANGE, 3)

/**
 * @brief 通信状态码枚举
 */
#define SX(name, value) COMMS_STATUS_##name = value,
typedef enum {
    COMMS_STATUS_TABLE
} CommsStatus;
#undef SX

/**
 * @brief WEED 包数据结构
 * @param id 识别到的杂草 ID
 * @param x 杂草在图像中的 X 坐标
 * @param y 杂草在图像中的 Y 坐标
 * @param confidence 识别置信度
 */
typedef struct {
    uint8_t id;
    uint16_t x;
    uint16_t y;
    float confidence;
} WeedData;

/**
 * @brief 通信单例接口
 */
#define SX(name, value) const CommsStatus name;
extern const struct CommsInterface {
    /**
     * @brief 通信状态码结构体
     */
    struct {
        COMMS_STATUS_TABLE
    };
    /**
     * @brief 处理数据包
     * @param packet 数据包指针
     * @param length 数据包长度
     * @return CommsStatus 状态码
     */
    CommsStatus(*process)(const uint8_t* packet, size_t length);
    /**
     * @brief 获取识别数据
     * @param data 输出的 WeedData 指针
     * @return CommsStatus 状态码
     */
    CommsStatus(*get_weed)(WeedData* data);
} s_comms_instance;
#undef SX

// ! ========================= 接 口 函 数 声 明 ========================= ! //

CommsStatus s_comms_process(const uint8_t* packet, size_t length);
CommsStatus s_comms_get_weed(WeedData* data);

#endif
