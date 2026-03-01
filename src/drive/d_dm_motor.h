#ifndef _d_dm_h_
#define _d_dm_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief DmMotorErrorCode_e 枚举类型，表示电机操作的结果
 * @param DM_MOTOR_SUCCESS 操作成功
 * @param DM_MOTOR_ERROR 操作失败
 * @param DM_MOTOR_TIMEOUT 操作超时
 * @param DM_MOTOR_ID_MISMATCH 电机 ID 不匹配
 */
typedef enum {
    DM_MOTOR_SUCCESS = 0,
    DM_MOTOR_ERROR,
    DM_MOTOR_TIMEOUT,
    DM_MOTOR_ID_MISMATCH,
} DmMotorErrorCode_e;

/**
 * @brief DmMotorMode_e 枚举类型，表示电机的工作模式
 * @param DM_MOTOR_MODE_MIT MIT 模式
 * @param DM_MOTOR_MODE_POS_SPD 位置速度模式
 * @param DM_MOTOR_MODE_SPD 速度模式
 * @param DM_MOTOR_MODE_POS_SPD_CUR 位置速度电流模式
 */
typedef enum {
    DM_MOTOR_MODE_MIT = 1,
    DM_MOTOR_MODE_POS_SPD = 2,
    DM_MOTOR_MODE_SPD = 3,
    DM_MOTOR_MODE_POS_SPD_CUR = 4,
} DmMotorMode_e;

/// @brief 电机控制命令的长度，单位为字节
#define DM_MOTOR_CMD_LEN                  8
/// @brief 电机反馈的超时时间，单位为毫秒
#define DM_MOTOR_FEEDBACK_TIMEOUT_MS      100

// ! ========================= 接 口 函 数 声 明 ========================= ! //

DmMotorErrorCode_e d_dm_enable(uint16_t id);
DmMotorErrorCode_e d_dm_disable(uint16_t id);

DmMotorErrorCode_e d_dm_set_mit(uint16_t id, float pos, float spd, float kp, float kd, float torque);
DmMotorErrorCode_e d_dm_set_pos_spd(uint16_t id, float pos, float spd);
DmMotorErrorCode_e d_dm_set_spd(uint16_t id, float spd);
DmMotorErrorCode_e d_dm_set_pos_spd_cur(uint16_t id, float pos, float spd, float cur);

DmMotorErrorCode_e d_dm_get_feedback(uint16_t id, uint8_t feedback[8], uint32_t timeout_ms);
DmMotorErrorCode_e d_dm_get_err_code(uint16_t id, uint8_t* err_code, uint32_t timeout_ms);
DmMotorErrorCode_e d_dm_get_pos(uint16_t id, float* pos, uint32_t timeout_ms);
DmMotorErrorCode_e d_dm_get_spd(uint16_t id, float* spd, uint32_t timeout_ms);
DmMotorErrorCode_e d_dm_get_torque(uint16_t id, float* torque, uint32_t timeout_ms);


#endif
