#include "d_dm_motor.h"
#include "d_can.h"

#include "d_systick.h"
#include "service/s_delay.h"

#include <stdio.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //



// ! ========================= 私 有 函 数 声 明 ========================= ! //

static DmMotorErrorCode_e dm_can_send(uint16_t id, uint8_t data[8]);
static DmMotorErrorCode_e dm_can_rcvd(uint8_t buffer[8]);
static void dm_write_register(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
static void dm_switch_mode(uint16_t id, DmMotorMode_e mode);
static uint16_t dm_f32_to_u16(float val, float min, float max, uint8_t bits);
static float dm_u16_to_f32(uint16_t val, float min, float max, uint8_t bits);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 使能电机
 * @param id 电机 ID
 * @return DmMotorErrorCode_e 枚举类型，表示操作结果
 */
DmMotorErrorCode_e d_dm_enable(uint16_t id) {
    uint8_t data[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };
    return dm_can_send(id, data);
}

/**
 * @brief 禁用电机
 * @param id 电机 ID
 * @return DmMotorErrorCode_e 枚举类型，表示操作结果
 */
DmMotorErrorCode_e d_dm_disable(uint16_t id) {
    uint8_t data[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD };
    return dm_can_send(id, data);
}

/**
 * @brief 设置电机为 MIT 模式并发送控制命令
 * @param id 电机 ID
 * @param pos 目标位置，单位为转，范围为 -12.5 到 12.5
 * @param spd 目标速度，单位为转每秒，范围为 -10 到 10
 * @param kp 位置环比例增益，范围为 0 到 500
 * @param kd 位置环微分增益，范围为 0 到 5
 * @param torque 目标电流，单位为 A，范围为 -28 到 28
 * @return DmMotorErrorCode_e 枚举类型，表示操作结果
 */
DmMotorErrorCode_e d_dm_set_mit(uint16_t id, float pos, float spd, float kp, float kd, float torque) {
    dm_switch_mode(id, DM_MOTOR_MODE_MIT);

    uint16_t pos_bits = dm_f32_to_u16(pos, -12.5f, 12.5f, 16);
    uint16_t spd_bits = dm_f32_to_u16(spd, -10.0f, 10.0f, 12);
    uint16_t kp_bits = dm_f32_to_u16(kp, 0.0f, 500.0f, 12);
    uint16_t kd_bits = dm_f32_to_u16(kd, 0.0f, 5.0f, 12);
    uint16_t torque_bits = dm_f32_to_u16(torque, -28.0f, 28.0f, 12);

    uint8_t data[8];
    data[0] = (uint8_t)(pos_bits >> 8);
    data[1] = (uint8_t)(pos_bits & 0xFF);
    data[2] = (uint8_t)(spd_bits >> 4);
    data[3] = (uint8_t)(((spd_bits & 0x0F) << 4) | (kp_bits >> 8));
    data[4] = (uint8_t)(kp_bits & 0xFF);
    data[5] = (uint8_t)(kd_bits >> 4);
    data[6] = (uint8_t)(((kd_bits & 0x0F) << 4) | (torque_bits >> 8));
    data[7] = (uint8_t)(torque_bits & 0xFF);

    return dm_can_send(id, data);
}

/**
 * @brief 设置电机为位置速度模式并发送控制命令
 * @param id 电机 ID
 * @param pos 目标位置，单位为转，范围为 -12.5 到 12.5
 * @param spd 目标速度，单位为转每秒，范围为 -10 到 10
 * @return DmMotorErrorCode_e 枚举类型，表示操作结果
 */
DmMotorErrorCode_e d_dm_set_pos_spd(uint16_t id, float pos, float spd) {
    dm_switch_mode(id, DM_MOTOR_MODE_POS_SPD);

    uint16_t target_id = (uint16_t)(id + 0x100);
    uint8_t data[8];
    memcpy(&data[0], &pos, sizeof(float));
    memcpy(&data[4], &spd, sizeof(float));

    return dm_can_send(target_id, data);
}

/**
 * @brief 设置电机为速度模式并发送控制命令
 * @param id 电机 ID
 * @param spd 目标速度，单位为转每秒，范围为 -10 到 10
 * @return DmMotorErrorCode_e 枚举类型，表示操作结果
 */
DmMotorErrorCode_e d_dm_set_spd(uint16_t id, float spd) {
    dm_switch_mode(id, DM_MOTOR_MODE_SPD);

    uint16_t target_id = (uint16_t)(id + 0x200);
    uint8_t data[8] = { 0 };
    memcpy(&data[0], &spd, sizeof(float));

    return dm_can_send(target_id, data);
}

/**
 * @brief 设置电机为位置速度电流模式并发送控制命令
 * @param id 电机 ID
 * @param pos 目标位置，单位为转，范围为 -12.5 到 12.5
 * @param spd 目标速度，单位为转每秒，范围为 -10 到 10
 * @param cur 目标电流，单位为 A，范围为 -28 到 28
 * @return DmMotorErrorCode_e 枚举类型，表示操作结果
 */
DmMotorErrorCode_e d_dm_set_pos_spd_cur(uint16_t id, float pos, float spd, float cur) {
    dm_switch_mode(id, DM_MOTOR_MODE_POS_SPD_CUR);

    uint16_t target_id = (uint16_t)(id + 0x300);
    uint16_t spd_scaled = (uint16_t)(spd * 100.0f);
    uint16_t cur_scaled = (uint16_t)(cur * 10000.0f);
    uint8_t data[8];

    memcpy(&data[0], &pos, sizeof(float));
    data[4] = (uint8_t)(spd_scaled & 0xFF);
    data[5] = (uint8_t)(spd_scaled >> 8);
    data[6] = (uint8_t)(cur_scaled & 0xFF);
    data[7] = (uint8_t)(cur_scaled >> 8);

    return dm_can_send(target_id, data);
}

/**
 * @brief 获取电机反馈信息
 * @param id 电机 ID
 * @param feedback 存储反馈数据的缓冲区，长度至少为 8 字节
 * @param timeout_ms 等待反馈的超时时间，单位为毫秒
 * @return DmMotorErrorCode_e 枚举类型，表示操作结果
 */
DmMotorErrorCode_e d_dm_get_feedback(uint16_t id, uint8_t feedback[8], uint32_t timeout_ms) {
    if(feedback == NULL) {
        return DM_MOTOR_ERROR;
    }

    uint8_t can_id_l = (uint8_t)(id & 0xFF);
    uint8_t can_id_h = (uint8_t)((id >> 8) & 0x07);
    uint8_t req_data[8] = { can_id_l, can_id_h, 0xCC, 0x00, 0, 0, 0, 0 };

    if(dm_can_send(0x7FF, req_data) != DM_MOTOR_SUCCESS) {
        return DM_MOTOR_ERROR;
    }

    ms_t start = d_systick_get_ms();
    while(!d_systick_is_timeout(start, timeout_ms)) {
        if(dm_can_rcvd(feedback) == DM_MOTOR_SUCCESS) {
            uint8_t expected_id = (uint8_t)(can_id_l & 0x0F);
            uint8_t received_id = (uint8_t)(feedback[0] & 0x0F);
            return (expected_id == received_id) ? DM_MOTOR_SUCCESS : DM_MOTOR_ID_MISMATCH;
        }
    }

    return DM_MOTOR_TIMEOUT;
}

/**
 * @brief 获取电机错误代码
 * @param id 电机 ID
 * @param err_code 存储错误代码的指针
 * @param timeout_ms 等待反馈的超时时间，单位为毫秒
 * @return DmMotorErrorCode_e 枚举类型，表示操作结果
 */
DmMotorErrorCode_e d_dm_get_err_code(uint16_t id, uint8_t* err_code, uint32_t timeout_ms) {
    if(err_code == NULL) {
        return DM_MOTOR_ERROR;
    }

    uint8_t feedback[8];
    DmMotorErrorCode_e result = d_dm_get_feedback(id, feedback, timeout_ms);
    if(result != DM_MOTOR_SUCCESS) {
        return result;
    }

    *err_code = (uint8_t)(feedback[0] >> 4);
    return DM_MOTOR_SUCCESS;
}

/**
 * @brief 获取电机位置
 * @param id 电机 ID
 * @param pos 存储位置的指针，单位为转
 * @param timeout_ms 等待反馈的超时时间，单位为毫秒
 * @return DmMotorErrorCode_e 枚举类型，表示操作结果
 */
DmMotorErrorCode_e d_dm_get_pos(uint16_t id, float* pos, uint32_t timeout_ms) {
    if(pos == NULL) {
        return DM_MOTOR_ERROR;
    }

    uint8_t feedback[8];
    DmMotorErrorCode_e result = d_dm_get_feedback(id, feedback, timeout_ms);

    if(result != DM_MOTOR_SUCCESS) {
        return result;
    }

    uint16_t pos_bits = (uint16_t)(((uint16_t)feedback[1] << 8) | feedback[2]);
    *pos = dm_u16_to_f32(pos_bits, -12.5f, 12.5f, 16);
    return DM_MOTOR_SUCCESS;
}

/**
 * @brief 获取电机速度
 * @param id 电机 ID
 * @param spd 存储速度的指针，单位为转每秒
 * @param timeout_ms 等待反馈的超时时间，单位为毫秒
 * @return DmMotorErrorCode_e 枚举类型，表示操作结果
 */
DmMotorErrorCode_e d_dm_get_spd(uint16_t id, float* spd, uint32_t timeout_ms) {
    if(spd == NULL) {
        return DM_MOTOR_ERROR;
    }

    uint8_t feedback[8];
    DmMotorErrorCode_e result = d_dm_get_feedback(id, feedback, timeout_ms);
    if(result != DM_MOTOR_SUCCESS) {
        return result;
    }

    uint16_t spd_bits = (uint16_t)(((uint16_t)feedback[3] << 4) | ((uint16_t)(feedback[4] & 0xF0) >> 4));
    *spd = dm_u16_to_f32(spd_bits, -10.0f, 10.0f, 12);
    return DM_MOTOR_SUCCESS;
}

/**
 * @brief 获取电机扭矩
 * @param id 电机 ID
 * @param torque 存储扭矩的指针，单位为 A
 * @param timeout_ms 等待反馈的超时时间，单位为毫秒
 * @return DmMotorErrorCode_e 枚举类型，表示操作结果
 */
DmMotorErrorCode_e d_dm_get_torque(uint16_t id, float* torque, uint32_t timeout_ms) {
    if(torque == NULL) {
        return DM_MOTOR_ERROR;
    }

    uint8_t feedback[8];
    DmMotorErrorCode_e result = d_dm_get_feedback(id, feedback, timeout_ms);
    if(result != DM_MOTOR_SUCCESS) {
        return result;
    }

    uint16_t torque_bits = (uint16_t)((((uint16_t)feedback[4] & 0x0F) << 8) | feedback[5]);
    *torque = dm_u16_to_f32(torque_bits, -28.0f, 28.0f, 12);
    return DM_MOTOR_SUCCESS;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 向 CAN 发送数据帧
 * @param id CAN ID
 * @param data 要发送的数据，长度必须为 8 字节
 * @return DmMotorErrorCode_e 枚举类型，表示操作结果
 */
static DmMotorErrorCode_e dm_can_send(uint16_t id, uint8_t data[8]) {
    if(data == NULL) {
        return DM_MOTOR_ERROR;
    }

    CanErrorCode_e can_result = d_can_write((can_instance_t* const)&g_canfd0, id, data, DM_MOTOR_CMD_LEN);
    s_delay_ms(1);
    return (can_result == CAN_SUCCESS) ? DM_MOTOR_SUCCESS : DM_MOTOR_ERROR;
}

/**
 * @brief 从 CAN 接收数据帧
 * @param buffer 存储接收数据的缓冲区，长度至少为 8 字节
 * @return DmMotorErrorCode_e 枚举类型，表示操作结果
 */
static DmMotorErrorCode_e dm_can_rcvd(uint8_t buffer[8]) {
    if(buffer == NULL) {
        return DM_MOTOR_ERROR;
    }

    can_frame_t frame;
    CanErrorCode_e can_result = d_can_read((can_instance_t* const)&g_canfd0, &frame);
    if(can_result != CAN_SUCCESS) {
        return DM_MOTOR_TIMEOUT;
    }

    uint8_t data_len = (uint8_t)frame.data_length_code;
    if(data_len > DM_MOTOR_CMD_LEN) {
        data_len = DM_MOTOR_CMD_LEN;
    }
    memcpy(buffer, frame.data, data_len);
    if(data_len < DM_MOTOR_CMD_LEN) {
        memset(&buffer[data_len], 0, DM_MOTOR_CMD_LEN - data_len);
    }

    return DM_MOTOR_SUCCESS;
}

/**
 * @brief 向电机写入寄存器
 * @param id 电机 ID
 * @param rid 寄存器 ID
 * @param d0 寄存器数据的第 0 字节
 * @param d1 寄存器数据的第 1 字节
 * @param d2 寄存器数据的第 2 字节
 * @param d3 寄存器数据的第 3 字节
 */
static void dm_write_register(uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3) {
    uint8_t can_id_l = (uint8_t)(id & 0xFF);
    uint8_t can_id_h = (uint8_t)((id >> 8) & 0x07);
    uint8_t data[8] = { can_id_l, can_id_h, 0x55, rid, d0, d1, d2, d3 };

    dm_can_send(0x7FF, data);
}

/**
 * @brief 切换电机工作模式
 * @param id 电机 ID
 * @param mode 目标工作模式
 */
static void dm_switch_mode(uint16_t id, DmMotorMode_e mode) {
    dm_write_register(id, 10, (uint8_t)mode, 0, 0, 0);
    s_delay_ms(1);
}

/**
 * @brief 将浮点数转换为无符号整数
 * @param val 要转换的浮点数
 * @param min 浮点数的最小值
 * @param max 浮点数的最大值
 * @param bits 转换后整数的位数
 * @return 转换后的无符号整数
 */
static uint16_t dm_f32_to_u16(float val, float min, float max, uint8_t bits) {
    if(bits == 0 || max <= min) {
        return 0;
    }

    if(val < min) {
        val = min;
    }
    if(val > max) {
        val = max;
    }

    float span = max - min;
    float normalized = (val - min) / span;
    uint32_t max_bits_val = (1UL << bits) - 1UL;

    return (uint16_t)(normalized * (float)max_bits_val);
}

/**
 * @brief 将无符号整数转换为浮点数
 * @param val 要转换的无符号整数
 * @param min 浮点数的最小值
 * @param max 浮点数的最大值
 * @param bits 转换前整数的位数
 * @return 转换后的浮点数
 */
static float dm_u16_to_f32(uint16_t val, float min, float max, uint8_t bits) {
    if(bits == 0 || max <= min) {
        return 0.0f;
    }

    float span = max - min;
    uint32_t max_bits_val = (1UL << bits) - 1UL;

    return ((float)val) * span / (float)max_bits_val + min;
}
