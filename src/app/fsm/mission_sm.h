#ifndef _mission_sm_h_
#define _mission_sm_h_

#include <stdbool.h>

#include "app/fsm/mission_context.h"

#include "device/motor.h"
#include "device/wifi_bt.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

#define mission mission_instance

/**
 * @brief 任务状态机事件枚举
 * @param NONE 无事件
 *
 * @param INIT_COMPLETE 初始化完成事件
 *
 * @param START_SEARCH 开始搜索事件
 * @param SEARCH_COMPLETE 搜索完成事件
 * @param MOVE_COMPLETE 运动完成事件
 * @param LASER_COMPLETE 激光完成事件
 * @param FINISH 任务完成事件
 * @param FINISH_COMPLETE 任务完成确认事件
 *
 * @param DETECT_ERROR 检测错误事件
 * @param ERROR_COMPLETE 错误处理完成事件
 */
#define EX(name) MISSION_EVENT_##name,
typedef enum {
#include "mission_def_event.inc"
} MissionEvent;
#undef EX

#define EX(name) .name = MISSION_EVENT_##name,
extern const struct MissionInstance {
    void(*init)(WifiBtConnectInfo* info);
    bool(*post)(MissionEvent event, void* data);
    void(*process)(void);
    void(*update_dm_feedback)(DmMotorFeedback feedback);

    const char* (*state)(void);
    const MissionContext* (*context)(void);
} mission_instance;
#undef EX

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void mission_sm_init(WifiBtConnectInfo* info);
bool mission_sm_post(MissionEvent event, void* data);
void mission_sm_process(void);
void mission_sm_update_dm_feedback(DmMotorFeedback feedback);

const char* mission_sm_state(void);
const MissionContext* mission_sm_context(void);

#endif
