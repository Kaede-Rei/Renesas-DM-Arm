#ifndef _fsm_h_
#define _fsm_h_

#include <stdbool.h>

#include "drive/d_wifi_bt.h"
#include "drive/d_dm_motor.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

#define FSM_DEPTH 3

/**
 * @brief FSM 状态表(name, parent)，由 X-Macro 定义，方便维护和扩展
 * null
 *  ├── normal
 *  │   ├── idle
 *  │   ├── searching
 *  │   ├── ik_pose
 *  │   ├── moving
 *  │   └── lasering
 *  │
 *  └── error
 */
#define STATE_TABLE \
    SX(init,        NULL)  \
    SX(normal,      NULL)  \
        SX(idle,        &state_normal)  \
        SX(searching,   &state_normal)  \
        SX(ik_pose,     &state_normal)  \
        SX(moving,      &state_normal)  \
        SX(lasering,    &state_normal)  \
    SX(error,       NULL)  \

/**
 * @brief FSM 事件表(name, value)，由 X-Macro 定义，方便维护和扩展
 * @param START_SEARCH 开始搜索事件
 * @param SEARCH_COMPLETE 搜索完成事件
 * @param IK_COMPLETE 逆运动学计算完成事件
 * @param MOVE_COMPLETE 运动完成事件
 * @param LASER_COMPLETE 激光执行完成事件
 * @param DETECT_ERROR 发现错误事件
 */
#define EVENT_TABLE \
    EX(NONE) \
    EX(INIT_COMPLETE) \
    EX(START_SEARCH) \
    EX(SEARCH_COMPLETE) \
    EX(IK_COMPLETE) \
    EX(MOVE_COMPLETE) \
    EX(LASER_COMPLETE) \
    EX(DETECT_ERROR) \
    EX(ERROR_COMPLETE) \

/**
 * @brief FSM 事件枚举类型，由 X-Macro 自动生成，表示状态机中的各种事件
 */
#define EX(name) EVENT_##name,
typedef enum {
    EVENT_TABLE
} Event;
#undef EX

/**
 * @brief FSM 事件消息结构体，包含事件类型和可选的事件数据指针
 * @param event 事件类型，使用 Event 枚举表示
 * @param data 事件数据指针，可以指向任何类型的数据，根据事件需要进行传递
 */
typedef struct {
    Event event;
    void* data;
} FsmMsg;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void fsm_init(WifiBtConnectInfo* info);
bool fsm_trigger(Event event, void* data);
void fsm_process(void);

void fsm_update_dm_feedback(const DmMotorFeedback feedback);

#endif
