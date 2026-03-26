#include "fsm.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <math.h>

#include "infra/matrix.h"

#include "platform/led.h"

#include "app/packet_parser.h"
#include "infra/delay.h"
#include "domain/arm_kine.h"

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief FSM 内部数据结构体，用于状态机全局数据存储
 * @param weed 当前检测到的杂草数据
 * @param error 当前错误信息字符串
 */
typedef struct {
    char* error;

    WifiBtConnectInfo* wifi_info;
    WeedData weed;
    Pose target_pose;
    SixDofJoint ik_result;
    DmMotorFeedback dm_fb[7];
    bool dm_fb_valid[7];
} FsmData;
static FsmData fsm_data;

/**
 * @brief FSM 状态结构体
 * @param name 状态名称
 * @param handle 事件处理函数指针，接受一个 Event 参数，返回下一个状态的指针
 * @param entry 状态进入动作函数指针，无参数无返回值
 * @param exit 状态退出动作函数指针，无参数无返回值
 * @param action 状态持续动作函数指针，无参数无返回值
 */
typedef struct State State;
struct State {
/// public:
    /**
     * @brief 状态名称，便于调试和日志记录
     */
    const char* name;
    /**
     * @brief 事件处理函数
     * @param event 事件
     */
    State* (*handle)(void);
    /**
     * @brief 状态进入函数
     */
    void (*entry)(void);
    /**
     * @brief 状态退出函数
     */
    void (*exit)(void);
    /**
     * @brief 状态持续函数
     */
    void (*action)(void);

/// private:
    /**
     * @brief 父状态指针，用于实现层次状态机，指向当前状态的父状态，如果没有父状态则为 NULL
     */
    State* _parent_;
};

/**
 * @brief FSM 当前状态和事件消息结构体，包含当前状态指针和当前事件消息
 * @param s 当前状态指针
 * @param msg 当前事件消息，包含事件类型和事件数据指针
 */
static struct {
    State* s;
    FsmMsg msg;
} cur;

// ! ========================= 状 态 机 声 明 ========================= ! //

#define SX(name, parent) \
    static State* handle_##name(void); \
    static void entry_##name(void); \
    static void exit_##name(void); \
    static void action_##name(void);
STATE_TABLE
#undef SX

#define SX(n, p) \
    static State state_##n = { \
        .name = #n, \
        .handle = handle_##n, \
        .entry = entry_##n, \
        .exit = exit_##n, \
        .action = action_##n, \
        ._parent_ = p \
    };
STATE_TABLE
#undef SX

static State* dispatch(void);
static State* find_lca(State* s1, State* s2);
static void exit_up_to(State* from, State* to);
static void enter_down_to(State* from, State* to);
static void execute_action(void);
static void reset_fsm_data(void);
static bool event_filter(Event event);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief FSM 初始化函数，设置初始状态和数据
 * @param info WiFi 连接信息指针，用于 FSM 内部使用
 */
void fsm_init(WifiBtConnectInfo* info) {
    reset_fsm_data();
    fsm_data.wifi_info = info;

    cur.msg.event = EVENT_NONE;
    cur.msg.data = NULL;
    cur.s = &state_init;

    cur.s->entry();
}

/**
 * @brief FSM 事件触发函数，接受一个事件和可选的数据指针，尝试触发事件并改变状态
 * @param event 事件类型，使用 Event 枚举表示
 * @param data 事件数据指针，可以指向任何类型的数据，根据事件需要进行传递
 * @return 如果事件成功触发并处理返回 true，否则返回 false（例如当前有未处理的事件或事件被过滤掉）
 */
bool fsm_trigger(Event event, void* data) {
    if(cur.msg.event != EVENT_NONE) return false;
    if(event_filter(event) == false) return false;

    cur.msg.event = event;
    cur.msg.data = data;

    return true;
}

/**
 * @brief FSM 处理函数，应该在主循环中定期调用，用于处理当前事件并执行状态转换和动作
 * @note 该函数会检查当前事件，调用状态的事件处理函数进行状态转换，并执行相应的进入、退出和持续动作
 */
void fsm_process(void) {
    if(cur.msg.event != EVENT_NONE) {
        State* next = dispatch();

        if(next != cur.s) {
            State* lca = find_lca(cur.s, next);
            exit_up_to(cur.s, lca);
            enter_down_to(lca, next);
            cur.s = next;
        }

        cur.msg.event = EVENT_NONE;
        cur.msg.data = NULL;
    }

    execute_action();
}

/**
 * @brief FSM 更新 DM 电机反馈数据函数，应该在接收到 DM 电机反馈时调用，用于更新 FSM 内部的电机反馈数据
 * @param feedback DM 电机反馈数据结构体，包含电机 ID、位置、速度等信息
 * @note 该函数会根据电机 ID 更新对应的电机反馈数据，并标记该数据为有效
 */
void fsm_update_dm_feedback(const DmMotorFeedback feedback) {
    if(feedback.id >= 1 && feedback.id <= 6) {
        fsm_data.dm_fb[feedback.id] = feedback;
        fsm_data.dm_fb_valid[feedback.id] = true;
    }
}

// ! ========================= 状 态 机 实 现 ========================= ! //

/**
 * @brief FSM 调度函数，负责根据当前事件在状态层次结构中查找处理该事件的状态，并返回下一个状态指针
 * @return 下一个状态指针，如果没有状态处理该事件则返回当前状态指针
 */
static State* dispatch(void) {
    State* s = cur.s;
    while(s) {
        if(s->handle) {
            State* next = s->handle();
            if(next) return next;
        }
        s = s->_parent_;
    }

    return cur.s;
}

/**
 * @brief 查找两个状态的最近公共祖先状态指针
 * @param s1 状态指针 1
 * @param s2 状态指针 2
 * @return 最近公共祖先状态指针，如果没有公共祖先则返回 NULL
 */
static State* find_lca(State* s1, State* s2) {
    if(!s1 || !s2) return NULL;

    int depth1 = 0, depth2 = 0;

    State* s = s1;
    while(s) { depth1++; s = s->_parent_; }
    s = s2;
    while(s) { depth2++; s = s->_parent_; }

    State* deeper = depth1 > depth2 ? s1 : s2;
    State* shallower = depth1 > depth2 ? s2 : s1;
    int diff = abs(depth1 - depth2);

    while(diff--) { deeper = deeper->_parent_; }
    while(deeper != shallower) {
        deeper = deeper->_parent_;
        shallower = shallower->_parent_;
    }

    return deeper;
}

/**
 * @brief 从当前状态退出到目标状态，调用沿途状态的退出函数
 * @param from 当前状态指针
 * @param to 目标状态指针
 */
static void exit_up_to(State* from, State* to) {
    State* s = from;
    while(s && s != to) {
        if(s->exit) s->exit();
        s = s->_parent_;
    }
}

/**
 * @brief 从目标状态进入到当前状态，调用沿途状态的进入函数
 * @param from 当前状态指针
 * @param to 目标状态指针
 */
static void enter_down_to(State* from, State* to) {
    State* path[FSM_DEPTH];
    int depth = 0;
    State* s = to;

    while(s && s != from && depth < FSM_DEPTH) {
        path[depth++] = s;
        s = s->_parent_;
    }

    while(depth--) {
        s = path[depth];
        if(s->entry) s->entry();
    }
}

/**
 * @brief 执行当前状态的持续动作函数，应该在状态机处理函数中定期调用
 * @note 该函数会从当前状态开始，沿着父状态链向上调用每个状态的持续动作函数，直到没有父状态为止
 */
static void execute_action(void) {
    State* s = cur.s;
    while(s) {
        if(s->action) s->action();
        s = s->_parent_;
    }
}

/**
 * @brief 重置 FSM 内部数据函数，应该在 FSM 初始化或进入特定状态时调用，用于清除 FSM 内部数据并重置为初始状态
 * @note 该函数会保留 WiFi 连接信息指针，但会清除其他数据字段
 */
static void reset_fsm_data(void) {
    WifiBtConnectInfo* wifi_info = fsm_data.wifi_info;
    fsm_data = (FsmData){ 0 };
    fsm_data.wifi_info = wifi_info;
}

/**
 * @brief FSM 事件过滤函数，应该在事件触发函数中调用，用于根据当前状态和事件类型决定是否允许触发该事件
 * @param event 事件类型，使用 Event 枚举表示
 * @return 如果事件被允许触发返回 true，否则返回 false（例如某些事件在特定状态下无效或被忽略）
 */
static bool event_filter(Event event) {
    State* s = cur.s;
    while(s) {
        if(s->handle) {
            Event old_event = cur.msg.event;
            cur.msg.event = event;

            State* next = s->handle();

            cur.msg.event = old_event;

            if(next) return true;
        }
        s = s->_parent_;
    }

    return false;
}

// ! ========================= 状 态 实 现 ========================= ! //


/**
 * @brief init 状态
 */
static State* handle_init(void) {
    switch(cur.msg.event) {
        case EVENT_INIT_COMPLETE:
            return &state_idle;
        default:
            return NULL;
    }
}
static void entry_init(void) {
    reset_fsm_data();

    fsm_trigger(EVENT_INIT_COMPLETE, NULL);
}
static void exit_init(void) {}
static void action_init(void) {}


/**
 * @brief normal 状态
 */
static State* handle_normal(void) {
    switch(cur.msg.event) {
        case EVENT_DETECT_ERROR:
            return &state_error;
        default:
            return NULL;
    }
}
static void entry_normal(void) {}
static void exit_normal(void) {}
static void action_normal(void) {}


/**
 * @brief idle 状态
 */
static State* handle_idle(void) {
    switch(cur.msg.event) {
        case EVENT_START_SEARCH:
            return &state_searching;
        default:
            return NULL;
    }
}
static void entry_idle(void) {
    printf("[FSM] 进入空闲状态，等待指令...\r\n");
}
static void exit_idle(void) {}
static void action_idle(void) {}


/**
 * @brief searching 状态
 */
static State* handle_searching(void) {
    switch(cur.msg.event) {
        case EVENT_SEARCH_COMPLETE:
            return &state_ik_pose;
        default:
            return NULL;
    }
}
static void entry_searching(void) {
    if(cur.msg.data) {
        fsm_data.weed = *(WeedData*)cur.msg.data;
        printf("[FSM] 捕获目标 ID：%d，坐标：x=%.2f, y=%.2f, z=%.2f, confidence=%.2f\r\n", fsm_data.weed.id, fsm_data.weed.x, fsm_data.weed.y, fsm_data.weed.z, fsm_data.weed.confidence);
    }
    else {
        fsm_trigger(EVENT_DETECT_ERROR, "未提供杂草数据");
    }
}
static void exit_searching(void) {}
static void action_searching(void) {
    Pose start_pose;
    // TODO: 搜索时使用的种子位姿
    // SixDofJoint seed_joints = { 0.0f, 0.1258f, 0.5978f, -0.4620f, 0.0f, 0.0f };

    start_pose.position.x = 0.0f;
    start_pose.position.y = 0.2f;
    start_pose.position.z = 0.3f;

    Vector3 aim_dir;
    aim_dir.x = fsm_data.weed.x - start_pose.position.x;
    aim_dir.y = fsm_data.weed.y - start_pose.position.y;
    aim_dir.z = fsm_data.weed.z - start_pose.position.z;

    float aim_norm;
    vec3_norm(&aim_dir, &aim_norm);
    if(aim_norm < 1e-8f) {
        fsm_trigger(EVENT_DETECT_ERROR, "目标位置无效");
        return;
    }

    vec3_normalize(&aim_dir, &aim_dir);
    Vector3 refer_dir = { 0.0f, 0.0f, 1.0f };
    float dot;
    vec3_dot(&aim_dir, &refer_dir, &dot);
    if(fabsf(dot) > 0.95f) { refer_dir = (Vector3){ 0.0f, 1.0f, 0.0f }; }

    Vector3 cross_x;
    vec3_cross(&aim_dir, &refer_dir, &cross_x);
    float cross_norm;
    vec3_norm(&cross_x, &cross_norm);
    if(cross_norm < 1e-8f) {
        fsm_trigger(EVENT_DETECT_ERROR, "瞄准姿态构造失败");
        return;
    }
    vec3_normalize(&cross_x, &cross_x);

    Vector3 cross_y;
    vec3_cross(&aim_dir, &cross_x, &cross_y);
    float cross_y_norm;
    vec3_norm(&cross_y, &cross_y_norm);
    if(cross_y_norm < 1e-8f) {
        fsm_trigger(EVENT_DETECT_ERROR, "瞄准姿态构造失败");
        return;
    }
    vec3_normalize(&cross_y, &cross_y);

    Matrix R; float R_data[3 * 3]; matrix(&R, 3, 3, R_data);
    R.pdata[0 * 3 + 0] = cross_x.x; R.pdata[0 * 3 + 1] = cross_y.x; R.pdata[0 * 3 + 2] = aim_dir.x;
    R.pdata[1 * 3 + 0] = cross_x.y; R.pdata[1 * 3 + 1] = cross_y.y; R.pdata[1 * 3 + 2] = aim_dir.y;
    R.pdata[2 * 3 + 0] = cross_x.z; R.pdata[2 * 3 + 1] = cross_y.z; R.pdata[2 * 3 + 2] = aim_dir.z;

    Quaternion quat;
    matrix_to_quat(&R, (float*)&quat);
    quat_normalize((float*)&quat, (float*)&quat);
    start_pose.orientation = quat;

    // TODO: 暂时只用 start_pose
    fsm_data.target_pose = start_pose;
    fsm_trigger(EVENT_SEARCH_COMPLETE, NULL);
}

/**
 * @brief ik_pose 状态
 */
static State* handle_ik_pose(void) {
    switch(cur.msg.event) {
        case EVENT_IK_COMPLETE:
            return &state_moving;
        default:
            return NULL;
    }
}
static void entry_ik_pose(void) {
    printf("[FSM] 进入 IK 计算，目标位姿：position=(%.2f, %.2f, %.2f), orientation=(w=%.2f, x=%.2f, y=%.2f, z=%.2f)\r\n",
        fsm_data.target_pose.position.x, fsm_data.target_pose.position.y, fsm_data.target_pose.position.z,
        fsm_data.target_pose.orientation.w, fsm_data.target_pose.orientation.x, fsm_data.target_pose.orientation.y, fsm_data.target_pose.orientation.z);
}
static void exit_ik_pose(void) {}
static void action_ik_pose(void) {
    SixDofJoint seed_joints = { 0.0f, 0.1258f, 0.5978f, -0.4620f, 0.0f, 0.0f };
    if(arm.ik(&fsm_data.target_pose, &fsm_data.ik_result, &seed_joints, IK_MODE_FULL_POSE) != ARM_STATUS_SUCCESS) {
        fsm_trigger(EVENT_DETECT_ERROR, "IK 计算失败");
        return;
    }
    printf("[FSM] IK 计算完成，关节角度：joint_1=%.2f, joint_2=%.2f, joint_3=%.2f, joint_4=%.2f, joint_5=%.2f, joint_6=%.2f\r\n", fsm_data.ik_result.joint_1, fsm_data.ik_result.joint_2, fsm_data.ik_result.joint_3, fsm_data.ik_result.joint_4, fsm_data.ik_result.joint_5, fsm_data.ik_result.joint_6);
    fsm_trigger(EVENT_IK_COMPLETE, NULL);
}


/**
 * @brief moving 状态
 */
static ms_t move_timer = 0;
static State* handle_moving(void) {
    switch(cur.msg.event) {
        case EVENT_MOVE_COMPLETE:
            return &state_lasering;
        default:
            return NULL;
    }
}
static void entry_moving(void) {
    printf("[FSM] 进入移动状态...\r\n");

    dm.set_pos_spd(0x01, fsm_data.ik_result.joint_1, 1.57f);
    dm.set_pos_spd(0x02, fsm_data.ik_result.joint_2, 1.57f);
    dm.set_pos_spd(0x03, fsm_data.ik_result.joint_3, 1.57f);
    dm.set_pos_spd(0x04, fsm_data.ik_result.joint_4, 1.57f);
    dm.set_pos_spd(0x05, fsm_data.ik_result.joint_5, 1.57f);
    dm.set_pos_spd(0x06, fsm_data.ik_result.joint_6, 1.57f);

    for(uint8_t i = 1; i <= 6; ++i) {
        fsm_data.dm_fb_valid[i] = false;
    }

    move_timer = 0;
}
static void exit_moving(void) {}
static void action_moving(void) {
    if(!s_nb_delay_ms(&move_timer, 5000)) {
        for(uint8_t i = 1; i <= 6; ++i) {
            if(!fsm_data.dm_fb_valid[i]) return;
        }

        float pos_err = 0.1f;
        if(fabsf(fsm_data.dm_fb[1].pos - fsm_data.ik_result.joint_1) > pos_err) return;
        if(fabsf(fsm_data.dm_fb[2].pos - fsm_data.ik_result.joint_2) > pos_err) return;
        if(fabsf(fsm_data.dm_fb[3].pos - fsm_data.ik_result.joint_3) > pos_err) return;
        if(fabsf(fsm_data.dm_fb[4].pos - fsm_data.ik_result.joint_4) > pos_err) return;
        if(fabsf(fsm_data.dm_fb[5].pos - fsm_data.ik_result.joint_5) > pos_err) return;
        if(fabsf(fsm_data.dm_fb[6].pos - fsm_data.ik_result.joint_6) > pos_err) return;

        fsm_trigger(EVENT_MOVE_COMPLETE, NULL);
    }

    fsm_trigger(EVENT_DETECT_ERROR, "移动超时");
}


/**
 * @brief lasering 状态
 */
static ms_t laser_timer = 0;
static State* handle_lasering(void) {
    switch(cur.msg.event) {
        case EVENT_LASER_COMPLETE:
            return &state_idle;
        default:
            return NULL;
    }
}
static void entry_lasering(void) {
    laser_timer = 0;
    wifi.send(*fsm_data.wifi_info, "Ready to Laser", strlen("Ready to Laser"));
}
static void exit_lasering(void) {}
static void action_lasering(void) {
    if(s_nb_delay_ms(&laser_timer, 5000)) {
        fsm_trigger(EVENT_DETECT_ERROR, "激光执行超时");
    }
}


/**
 * @brief error 状态
 */
static State* handle_error(void) {
    switch(cur.msg.event) {
        case EVENT_ERROR_COMPLETE:
            return &state_init;
        default:
            return NULL;
    }
}
static void entry_error(void) {
    if(cur.msg.data) {
        fsm_data.error = (char*)cur.msg.data;
        printf("[FSM] 错误发生：%s\r\n", fsm_data.error);
    }
}
static void exit_error(void) {}
static void action_error(void) {
    printf("[FSM] 错误状态，等待重置...\r\n");
    while(1) {
        led.toggle();
        delay_ms(500);
    }
}
