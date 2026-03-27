#ifndef _mission_context_h_
#define _mission_context_h_

#include <stdbool.h>

#include "app/packet_parser.h"

#include "device/motor.h"
#include "device/wifi_bt.h"

#include "domain/arm_kine.h"

#include "infra/delay.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief mission 状态机业务上下文
 */
typedef struct {
    /**
     * @brief 当前错误信息
     */
    const char* error;

    /**
     * @brief WiFi 连接信息
     */
    WifiBtConnectInfo* wifi_info;

    /**
     * @brief 当前目标杂草数据
     */
    WeedData weed;

    /**
     * @brief 目标位姿
     */
    Pose target_pose;

    /**
     * @brief IK 求解结果
     */
    SixDofJoint ik_result;

    /**
     * @brief 电机反馈缓存，索引按 id 使用（1~6）
     */
    DmMotorFeedback dm_fb[7];

    /**
     * @brief 电机反馈有效标记
     */
    bool dm_fb_valid[7];

    /**
     * @brief moving 状态计时器
     */
    ms_t move_timer;

    /**
     * @brief lasering 状态计时器
     */
    ms_t laser_timer;

    /**
     * @brief finish 状态计时器
     */
    ms_t finish_timer;
} MissionContext;

#endif