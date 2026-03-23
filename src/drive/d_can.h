#ifndef _d_can_h_
#define _d_can_h_

#include "hal_data.h"   // IWYU pragma: keep
#include "r_can_api.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief can0 写入数据的宏定义，简化函数调用
 * @param id CAN ID
 * @param data 要发送的数据
 * @param length 要发送的数据长度
 */
#define d_can0_write(id, data, length) d_can_write(&g_canfd0, id, data, length)
/**
 * @brief can0 读取数据的宏定义，简化函数调用
 * @param frame 指向存储接收数据的 CAN 帧结构体的指针
 */
#define d_can0_read(frame) d_can_read(&g_canfd0, frame)

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
extern const struct can_interface {
    CanErrorCode_e(*init)(void);
    CanErrorCode_e(*tx_complete)(void);
    CanErrorCode_e(*rx_complete)(void);
    CanErrorCode_e(*is_busy)(void);
    CanErrorCode_e(*write)(can_instance_t* const can_instance, uint32_t id, uint8_t* data, uint8_t length);
    CanErrorCode_e(*read)(can_instance_t* const can_instance, can_frame_t* const frame);
} can;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

CanErrorCode_e d_can_init(void);
CanErrorCode_e d_can_tx_complete(void);
CanErrorCode_e d_can_rx_complete(void);
CanErrorCode_e d_can_is_busy(void);
CanErrorCode_e d_can_write(can_instance_t* const can_instance, uint32_t id, uint8_t* data, uint8_t length);
CanErrorCode_e d_can_read(can_instance_t* const can_instance, can_frame_t* const frame);

#endif
