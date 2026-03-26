#ifndef _motor_h_
#define _motor_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 电机单例用户自定义名称
 */
#define dm d_dm_motor_instance

/**
 * @brief 达妙电机状态码表，使用 X-Macro 定义，方便维护和扩展
 * @param OK 操作成功
 * @param ERROR 操作失败
 * @param TIMEOUT 操作超时
 * @param ID_MISMATCH 电机 ID 不匹配
 */
#define DM_MOTOR_STATUS_TABLE \
    X(OK, 0) \
    X(ERROR, 1) \
    X(TIMEOUT, 2) \
    X(ID_MISMATCH, 3)

/**
 * @brief 达妙电机模式表，使用 X-Macro 定义，方便维护和扩展
 * @param MIT MIT 模式
 * @param POS_SPD 位置速度模式
 * @param SPD 速度模式
 * @param POS_SPD_CUR 位置速度电流模式
 */
#define DM_MOTOR_MODE_TABLE \
    Y(MIT, 1) \
    Y(POS_SPD, 2) \
    Y(SPD, 3) \
    Y(POS_SPD_CUR, 4)

/**
 * @brief 达妙电机状态码，由 X-Macro 自动生成枚举类型
 */
#define X(name, value) DM_MOTOR_##name = value,
typedef enum {
    DM_MOTOR_STATUS_TABLE
} DmMotorStatus;
#undef X

/**
 * @brief DmMotorMode 枚举类型，表示电机的工作模式
 * @param DM_MOTOR_MODE_MIT MIT 模式
 * @param DM_MOTOR_MODE_POS_SPD 位置速度模式
 * @param DM_MOTOR_MODE_SPD 速度模式
 * @param DM_MOTOR_MODE_POS_SPD_CUR 位置速度电流模式
 */
#define Y(name, value) DM_MOTOR_MODE_##name = value,
typedef enum {
    DM_MOTOR_MODE_TABLE
} DmMotorMode;
#undef Y

/**
 * @brief 电机反馈信息结构体
 * @param id 电机 ID
 * @param err_code 错误代码
 * @param pos 位置，单位为弧度
 * @param spd 速度，单位为弧度每秒
 * @param torque 扭矩，单位为 A
 */
typedef struct {
    uint16_t id;
    uint8_t err_code;
    float pos;
    float spd;
    float torque;
} DmMotorFeedback;

/// @brief 电机控制命令的长度，单位为字节
#define DM_MOTOR_CMD_LEN                8
/// @brief pos 上下限
#define DM_MOTOR_POS_LIMIT              12.5f
/// @brief spd 上下限
#define DM_MOTOR_SPD_LIMIT              10.0f
/// @brief torque 上下限
#define DM_MOTOR_TORQUE_LIMIT           28.0f
/// @brief kp 上限
#define DM_MOTOR_KP_LIMIT               500.0f
/// @brief kd 上限
#define DM_MOTOR_KD_LIMIT               5.0f

/**
 * @brief 电机接口结构体，包含所有电机相关的函数指针
 * @param enable 使能电机
 * @param disable 禁用电机
 * @param set_mit 设置 MIT 模式控制命令
 * @param set_pos_spd 设置位置速度模式控制命令
 * @param set_spd 设置速度模式控制命令
 * @param set_pos_spd_cur 设置位置速度电流模式控制命令
 * @param get_feedback 获取电机反馈信息
 * @param get_err_code 获取电机错误代码
 * @param get_pos 获取电机位置
 * @param get_spd 获取电机速度
 * @param get_torque 获取电机扭矩
 * @param request_feedback 请求电机发送反馈信息
 * @param update 更新电机反馈信息
 */
#define X(name, value) const DmMotorStatus name;
#define Y(name, value) const DmMotorMode name;
extern const struct DmMotorInterface {
    /**
     * @brief 电机状态码 DmMotorStatus，由 X-Macro 自动生成枚举类型
     * @param OK 操作成功
     * @param ERROR 操作失败
     * @param TIMEOUT 操作超时
     * @param ID_MISMATCH 电机 ID 不匹配
     */
    struct {
        DM_MOTOR_STATUS_TABLE
    };
    /**
     * @brief 电机模式 DmMotorMode，由 X-Macro 自动生成枚举类型
     * @param MIT MIT 模式
     * @param POS_SPD 位置速度模式
     * @param SPD 速度模式
     * @param POS_SPD_CUR 位置速度电流模式
     */
    struct {
        DM_MOTOR_MODE_TABLE
    };
    /**
     * @brief 将电机状态码转换为字符串
     * @param status 电机状态码
     */
    const char* (*status_str)(DmMotorStatus status);
    /**
     * @brief 将电机模式转换为字符串
     * @param mode 电机模式
     */
    const char* (*mode_str)(DmMotorMode mode);
    /**
     * @brief 使能电机
     * @param id 电机 ID
     * @return DmMotorStatus 枚举类型，表示操作结果
     */
    DmMotorStatus(*enable)(uint16_t id);

    /**
     * @brief 禁用电机
     * @param id 电机 ID
     * @return DmMotorStatus 枚举类型，表示操作结果
     */
    DmMotorStatus(*disable)(uint16_t id);

    /**
     * @brief 设置 MIT 模式
     * @param id 电机 ID
     * @param pos 目标位置
     * @param spd 目标速度
     * @param kp 位置环比例增益
     * @param kd 位置环微分增益
     * @param torque 目标电流
     * @return DmMotorStatus 枚举类型，表示操作结果
     */
    DmMotorStatus(*set_mit)(uint16_t id, float pos, float spd, float kp, float kd, float torque);

    /**
     * @brief 设置位置速度模式
     * @param id 电机 ID
     * @param pos 目标位置
     * @param spd 目标速度
     * @return DmMotorStatus 枚举类型，表示操作结果
     */
    DmMotorStatus(*set_pos_spd)(uint16_t id, float pos, float spd);

    /**
     * @brief 设置速度模式
     * @param id 电机 ID
     * @param spd 目标速度
     * @return DmMotorStatus 枚举类型，表示操作结果
     */
    DmMotorStatus(*set_spd)(uint16_t id, float spd);

    /**
     * @brief 设置位置速度电流模式
     * @param id 电机 ID
     * @param pos 目标位置
     * @param spd 目标速度
     * @param cur 目标电流
     * @return DmMotorStatus 枚举类型，表示操作结果
     */
    DmMotorStatus(*set_pos_spd_cur)(uint16_t id, float pos, float spd, float cur);

    /**
     * @brief 获取电机原始反馈数据
     * @param id 电机 ID
     * @param feedback 反馈数据数组，长度为8
     * @param timeout_ms 超时时间（毫秒）
     * @return DmMotorStatus 枚举类型，表示操作结果
     */
    DmMotorStatus(*get_feedback)(uint16_t id, uint8_t feedback[8], uint32_t timeout_ms);

    /**
     * @brief 获取电机错误码
     * @param id 电机 ID
     * @param err_code 错误码指针
     * @param timeout_ms 超时时间（毫秒）
     * @return DmMotorStatus 枚举类型，表示操作结果
     */
    DmMotorStatus(*get_err_code)(uint16_t id, uint8_t* err_code, uint32_t timeout_ms);

    /**
     * @brief 获取电机位置
     * @param id 电机 ID
     * @param pos 位置指针
     * @param timeout_ms 超时时间（毫秒）
     * @return DmMotorStatus 枚举类型，表示操作结果
     */
    DmMotorStatus(*get_pos)(uint16_t id, float* pos, uint32_t timeout_ms);

    /**
     * @brief 获取电机速度
     * @param id 电机 ID
     * @param spd 速度指针
     * @param timeout_ms 超时时间（毫秒）
     * @return DmMotorStatus 枚举类型，表示操作结果
     */
    DmMotorStatus(*get_spd)(uint16_t id, float* spd, uint32_t timeout_ms);

    /**
     * @brief 获取电机扭矩
     * @param id 电机 ID
     * @param torque 扭矩指针
     * @param timeout_ms 超时时间（毫秒）
     * @return DmMotorStatus 枚举类型，表示操作结果
     */
    DmMotorStatus(*get_torque)(uint16_t id, float* torque, uint32_t timeout_ms);

    /**
     * @brief 主动请求电机反馈
     * @param id 电机 ID
     * @return DmMotorStatus 枚举类型，表示操作结果
     */
    DmMotorStatus(*request_feedback)(uint16_t id);

    /**
     * @brief 更新电机反馈结构体
     * @param feedback 电机反馈结构体指针
     * @return DmMotorStatus 枚举类型，表示操作结果
     */
    DmMotorStatus(*update)(DmMotorFeedback* feedback);
} d_dm_motor_instance;
#undef X
#undef Y

// ! ========================= 接 口 函 数 声 明 ========================= ! //

const char* d_dm_status_str(DmMotorStatus status);
const char* d_dm_mode_str(DmMotorMode mode);

DmMotorStatus d_dm_enable(uint16_t id);
DmMotorStatus d_dm_disable(uint16_t id);

DmMotorStatus d_dm_set_mit(uint16_t id, float pos, float spd, float kp, float kd, float torque);
DmMotorStatus d_dm_set_pos_spd(uint16_t id, float pos, float spd);
DmMotorStatus d_dm_set_spd(uint16_t id, float spd);
DmMotorStatus d_dm_set_pos_spd_cur(uint16_t id, float pos, float spd, float cur);

DmMotorStatus d_dm_get_feedback(uint16_t id, uint8_t feedback[8], uint32_t timeout_ms);
DmMotorStatus d_dm_get_err_code(uint16_t id, uint8_t* err_code, uint32_t timeout_ms);
DmMotorStatus d_dm_get_pos(uint16_t id, float* pos, uint32_t timeout_ms);
DmMotorStatus d_dm_get_spd(uint16_t id, float* spd, uint32_t timeout_ms);
DmMotorStatus d_dm_get_torque(uint16_t id, float* torque, uint32_t timeout_ms);

DmMotorStatus d_dm_request_feedback(uint16_t id);
DmMotorStatus d_dm_update(DmMotorFeedback* feedback);

#endif
