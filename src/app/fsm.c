#include "app/fsm.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#include "fsm.h"
#include "tools/matrix.h"

#include "drive/d_led.h"

#include "service/s_comms.h"
#include "service/s_delay.h"
#include "service/s_fk_ik.h"

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

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static bool generate_aim_pose(Vector3 source, Vector3 target, Pose* out);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void fsm_init(WifiBtConnectInfo* info) {
    reset_fsm_data();
    fsm_data.wifi_info = info;

    cur.msg.event = EVENT_NONE;
    cur.msg.data = NULL;
    cur.s = &state_init;

    cur.s->entry();
}

bool fsm_trigger(Event event, void* data) {
    if(cur.msg.event != EVENT_NONE) return false;
    if(event_filter(event) == false) return false;

    cur.msg.event = event;
    cur.msg.data = data;

    return true;
}

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

void fsm_update_dm_feedback(const DmMotorFeedback feedback) {
    if(feedback.id >= 1 && feedback.id <= 6) {
        fsm_data.dm_fb[feedback.id] = feedback;
        fsm_data.dm_fb_valid[feedback.id] = true;
    }
}

// ! ========================= 状 态 机 实 现 ========================= ! //

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

static void exit_up_to(State* from, State* to) {
    State* s = from;
    while(s && s != to) {
        if(s->exit) s->exit();
        s = s->_parent_;
    }
}

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

static void execute_action(void) {
    State* s = cur.s;
    while(s) {
        if(s->action) s->action();
        s = s->_parent_;
    }
}

static void reset_fsm_data(void) {
    WifiBtConnectInfo* wifi_info = fsm_data.wifi_info;
    fsm_data = (FsmData){ 0 };
    fsm_data.wifi_info = wifi_info;
}

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
        case EVENT_FINISH:
            return &state_finish;
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
            return &state_moving;
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
    Vector3 ideal_pos = { 0.0f, 0.2f, 0.3f };
    Vector3 target_pos = { fsm_data.weed.x, fsm_data.weed.y, fsm_data.weed.z };
    SixDofJoint seed_joints = { 0.0f, 0.6f, 1.14f, -1.06f, 0.0f, 0.0f };

    float offsets[] = { 0.0f, 0.02f, -0.02f, 0.05f, -0.05f };
    int num_offsets = sizeof(offsets) / sizeof(offsets[0]);

    bool found = false;
    Pose best_pose;

    printf("[FSM] 开始瞄准位姿搜索算法，目标:(%.2f, %.2f, %.2f)...\r\n", target_pos.x, target_pos.y, target_pos.z);

    for(int i = 0; i < num_offsets && !found; i++) {
        for(int j = 0; j < num_offsets && !found; j++) {
            for(int k = 0; k < num_offsets && !found; k++) {

                Vector3 search_pos = ideal_pos;
                search_pos.x += offsets[i];
                search_pos.y += offsets[j];
                search_pos.z += offsets[k];

                Pose candidate_pose;
                if(!generate_aim_pose(search_pos, target_pos, &candidate_pose)) {
                    continue;
                }
                if(arm.ik(&candidate_pose, &fsm_data.ik_result, &seed_joints, IK_MODE_FULL_POSE) == ARM_STATUS_SUCCESS) {
                    best_pose = candidate_pose;
                    found = true;
                }
            }
        }
    }

    if(found) {
        fsm_data.target_pose = best_pose;

        printf("[FSM] IK 搜索成功！采用发射点: (%.2f, %.2f, %.2f)\r\n",
            best_pose.position.x, best_pose.position.y, best_pose.position.z);
        printf("[FSM] 关节解: J1=%.2f, J2=%.2f, J3=%.2f, J4=%.2f, J5=%.2f, J6=%.2f\r\n",
            fsm_data.ik_result.joint_1, fsm_data.ik_result.joint_2, fsm_data.ik_result.joint_3,
            fsm_data.ik_result.joint_4, fsm_data.ik_result.joint_5, fsm_data.ik_result.joint_6);

        fsm_trigger(EVENT_SEARCH_COMPLETE, NULL);
    }
    else {
        printf("[FSM] 警告：IK 空间搜索失败，此坐标不可达\r\n");
        fsm_trigger(EVENT_DETECT_ERROR, "IK 网格搜索无解");
    }
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
        // for(uint8_t i = 1; i <= 6; ++i) {
        //     if(!fsm_data.dm_fb_valid[i]) return;
        // }

        // float pos_err = 0.1f;
        // if(fabsf(fsm_data.dm_fb[1].pos - fsm_data.ik_result.joint_1) > pos_err) return;
        // if(fabsf(fsm_data.dm_fb[2].pos - fsm_data.ik_result.joint_2) > pos_err) return;
        // if(fabsf(fsm_data.dm_fb[3].pos - fsm_data.ik_result.joint_3) > pos_err) return;
        // if(fabsf(fsm_data.dm_fb[4].pos - fsm_data.ik_result.joint_4) > pos_err) return;
        // if(fabsf(fsm_data.dm_fb[5].pos - fsm_data.ik_result.joint_5) > pos_err) return;
        // if(fabsf(fsm_data.dm_fb[6].pos - fsm_data.ik_result.joint_6) > pos_err) return;

        printf("[FSM] 移动完成，进入激光状态...\r\n");
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
static void exit_lasering(void) {
    printf("[FSM] 激光完成，返回空闲状态...\r\n");
}
static void action_lasering(void) {
    if(s_nb_delay_ms(&laser_timer, 5000)) {
        fsm_trigger(EVENT_DETECT_ERROR, "激光执行超时");
    }
}


/**
 * @brief finish 状态
 */
static State* handle_finish(void) {
    switch(cur.msg.event) {
        case EVENT_FINISH_COMPLETE:
            return &state_idle;
        default:
            return NULL;
    }
}
static void entry_finish(void) {
    printf("[FSM] 进入完成状态...\r\n");
    dm.set_pos_spd(0x01, 0.0f, 1.57f);
    dm.set_pos_spd(0x02, 0.0f, 1.57f);
    dm.set_pos_spd(0x03, 0.0f, 1.57f);
    dm.set_pos_spd(0x04, 0.0f, 1.57f);
    dm.set_pos_spd(0x05, 0.0f, 1.57f);
    dm.set_pos_spd(0x06, 0.0f, 1.57f);

    for(uint8_t i = 1; i <= 6; ++i) {
        fsm_data.dm_fb_valid[i] = false;
    }

    move_timer = 0;
}
static void exit_finish(void) {}
static void action_finish(void) {
    if(!s_nb_delay_ms(&move_timer, 10000)) {
        // for(uint8_t i = 1; i <= 6; ++i) {
        //     if(!fsm_data.dm_fb_valid[i]) return;
        // }

        // float pos_err = 0.1f;
        // if(fabsf(fsm_data.dm_fb[1].pos - 0.0f) > pos_err) return;
        // if(fabsf(fsm_data.dm_fb[2].pos - 0.0f) > pos_err) return;
        // if(fabsf(fsm_data.dm_fb[3].pos - 0.0f) > pos_err) return;
        // if(fabsf(fsm_data.dm_fb[4].pos - 0.0f) > pos_err) return;
        // if(fabsf(fsm_data.dm_fb[5].pos - 0.0f) > pos_err) return;
        // if(fabsf(fsm_data.dm_fb[6].pos - 0.0f) > pos_err) return;

        printf("[FSM] 复位完成\r\n");
        fsm_trigger(EVENT_FINISH_COMPLETE, NULL);
    }
    fsm_trigger(EVENT_DETECT_ERROR, "复位超时");
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
        s_delay_ms(500);
    }
}

static bool generate_aim_pose(Vector3 source, Vector3 target, Pose* out) {
    out->position.x = source.x;
    out->position.y = source.y;
    out->position.z = source.z;

    Vector3 aim_dir = { target.x - source.x, target.y - source.y, target.z - source.z };
    float aim_norm;
    vec3_norm(&aim_dir, &aim_norm);
    if(aim_norm < 1e-4f) return false;
    vec3_normalize(&aim_dir, &aim_dir);
    Vector3 z_axis = aim_dir;

    Vector3 world_up = { 0.0f, 0.0f, 1.0f };
    Vector3 x_axis;
    vec3_cross(&world_up, &z_axis, &x_axis);
    float x_norm;
    vec3_norm(&x_axis, &x_norm);

    if(x_norm < 1e-3f) {
        world_up = (Vector3){ 0.0f, 1.0f, 0.0f };
        vec3_cross(&world_up, &z_axis, &x_axis);
        vec3_norm(&x_axis, &x_norm);
    }
    vec3_normalize(&x_axis, &x_axis);

    Vector3 y_axis;
    vec3_cross(&z_axis, &x_axis, &y_axis);
    vec3_normalize(&y_axis, &y_axis);

    Matrix R; float R_data[3 * 3]; matrix(&R, 3, 3, R_data);
    R.pdata[0 * 3 + 0] = x_axis.x; R.pdata[0 * 3 + 1] = y_axis.x; R.pdata[0 * 3 + 2] = z_axis.x;
    R.pdata[1 * 3 + 0] = x_axis.y; R.pdata[1 * 3 + 1] = y_axis.y; R.pdata[1 * 3 + 2] = z_axis.y;
    R.pdata[2 * 3 + 0] = x_axis.z; R.pdata[2 * 3 + 1] = y_axis.z; R.pdata[2 * 3 + 2] = z_axis.z;

    Quaternion quat;
    matrix_to_quat(&R, (float*)&quat);
    quat_normalize((float*)&quat, (float*)&quat);
    out->orientation = quat;

    return true;
}
