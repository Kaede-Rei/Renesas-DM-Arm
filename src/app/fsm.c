#include "app/fsm.h"

#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <math.h>

#include "drive/d_wifi_bt.h"

#include "service/s_comms.h"
#include "service/s_delay.h"

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief FSM 内部数据结构体，用于状态机全局数据存储
 * @param weed 当前检测到的杂草数据
 * @param error 当前错误信息字符串
 */
typedef struct {
    char* error;

    WeedData weed;
    WifiBtConnectInfo wifi_info;
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

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void fsm_init(void) {
    reset_fsm_data();

    cur.msg.event = 0;
    cur.msg.data = NULL;
    cur.s = &state_init;

    cur.s->entry();
}

void fsm_trigger(Event event, void* data) {
    cur.msg.event = event;
    cur.msg.data = data;
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

    return s;
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
    int diff = (int)fabs((double)(depth1 - depth2));

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
    fsm_data = (FsmData){ 0 };
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
static void entry_idle(void) {}
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
}
static void exit_searching(void) {}
static void action_searching(void) {

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
static void entry_ik_pose(void) {}
static void exit_ik_pose(void) {}
static void action_ik_pose(void) {}


/**
 * @brief moving 状态
 */
static State* handle_moving(void) {
    switch(cur.msg.event) {
        case EVENT_MOVE_COMPLETE:
            return &state_lasering;
        default:
            return NULL;
    }
}
static void entry_moving(void) {}
static void exit_moving(void) {}
static void action_moving(void) {}


/**
 * @brief lasering 状态
 */
static State* handle_lasering(void) {
    switch(cur.msg.event) {
        case EVENT_LASER_COMPLETE:
            return &state_idle;
        default:
            return NULL;
    }
}
static void entry_lasering(void) {
    if(cur.msg.data) {
        fsm_data.wifi_info = *(WifiBtConnectInfo*)cur.msg.data;
        wifi.send(fsm_data.wifi_info, "Ready to Laser", strlen("Ready to Laser"));
    }

}
static void exit_lasering(void) {}
static void action_lasering(void) {
    static ms_t laser_timer = 0;
    if(s_nb_delay_ms(&laser_timer, 5000)) {
        fsm_trigger(EVENT_LASER_COMPLETE, NULL);
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
static void action_error(void) {}
