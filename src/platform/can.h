#ifndef _can_h_
#define _can_h_

#include "hal_data.h"   // IWYU pragma: keep

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief CAN 单例用户自定义名称
 */
#define can can_instance

/**
 * @brief can0 写入数据的宏定义，简化函数调用
 * @param id CAN ID
 * @param data 要发送的数据
 * @param length 要发送的数据长度
 */
#define can0_write(id, data, length) can_write(&g_canfd0, id, data, length)
/**
 * @brief can0 读取数据的宏定义，简化函数调用
 * @param frame 指向存储接收数据的 CAN 帧结构体的指针
 */
#define can0_read(frame) can_read(&g_canfd0, frame)

/**
 * @brief 官方要求用户必须定义一个全局的 CANFD 接收过滤器数组
 */
extern const canfd_afl_entry_t p_canfd0_afl[CANFD_CFG_AFL_CH0_RULE_NUM];

/**
 * @brief CAN 错误代码枚举类型
 * @param CAN_SUCCESS 操作成功
 * @param CAN_ERROR 操作失败
 * @param CAN_NOT_COMPLETE 操作未完成
 * @param CAN_BUSY CAN 正在忙碌
 * @param CAN_OVERFLOW 接收缓冲区溢出
 * @param CAN_EMPTY 接收缓冲区为空
 */
typedef enum {
    CAN_SUCCESS = 0,
    CAN_ERROR,
    CAN_NOT_COMPLETE,
    CAN_BUSY,
    CAN_OVERFLOW,
    CAN_EMPTY,
} CanErrorCode_e;

/**
 * @brief CAN 单例
 * @param init 初始化函数指针
 * @param tx_complete 发送完成函数指针
 * @param rx_complete 接收完成函数指针
 * @param is_busy 判断是否忙碌函数指针
 * @param write 写入数据函数指针
 * @param read 读取数据函数指针
 */
extern const struct CanInterface {
    /**
     * @brief 初始化 CAN 模块
     * @return CanErrorCode_e 枚举类型，表示操作结果
     */
    CanErrorCode_e(*init)(void);
    /**
     * @brief 检查 CAN 发送是否完成
     * @return CanErrorCode_e 枚举类型，表示操作结果
     */
    CanErrorCode_e(*tx_complete)(void);
    /**
     * @brief 检查 CAN 接收是否完成
     * @return CanErrorCode_e 枚举类型，表示操作结果
     */
    CanErrorCode_e(*rx_complete)(void);
    /**
     * @brief 检查 CAN 是否忙碌
     * @return CanErrorCode_e 枚举类型，表示操作结果
     */
    CanErrorCode_e(*is_busy)(void);
    /**
     * @brief 向 CAN 发送数据帧
     * @param can_instance 指向 CAN 实例的指针
     * @param frame 指向要发送的 CAN 帧的指针
     * @note 此函数使用 CAN 而非 CANFD
     */
    CanErrorCode_e(*write)(can_instance_t* const can_instance, uint32_t id, uint8_t* data, uint8_t length);
    /**
     * @brief 从 CAN 接收数据帧
     * @param can_instance 指向 CAN 实例的指针
     * @param frame 指向存储接收数据的 CAN 帧结构体的指针
     * @return CanErrorCode_e 枚举类型，表示操作结果
     * @note 此函数使用 CAN 而非 CANFD
     */
    CanErrorCode_e(*read)(can_instance_t* const can_instance, can_frame_t* const frame);
} can_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

CanErrorCode_e can_init(void);
CanErrorCode_e can_tx_complete(void);
CanErrorCode_e can_rx_complete(void);
CanErrorCode_e can_is_busy(void);
CanErrorCode_e can_write(can_instance_t* const can_instance, uint32_t id, uint8_t* data, uint8_t length);
CanErrorCode_e can_read(can_instance_t* const can_instance, can_frame_t* const frame);

#endif
