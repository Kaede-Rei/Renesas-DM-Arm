#include "app/fsm.h"

#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <math.h>

#include "drive/d_dm_motor.h"

// #include "service/s_delay.h"

// ! ========================= 变 量 声 明 ========================= ! //

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

static struct {
    WifiBtConnectInfo info;
} fsm_data;

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

// ! ========================= 接 口 函 数 实 现 ========================= ! //

#define SX(n, p) (void)state_##n;
void fsm_init(WifiBtConnectInfo* info) {
    cur.msg.event = 0;
    cur.msg.data = NULL;
    cur.s = &state_init;

    fsm_data.info = *info;

    STATE_TABLE
}
#undef SX

void fsm_trigger(Event event, void* data) {
    cur.msg.event = event;
    cur.msg.data = data;
}


void fsm_process(void) {
    if(cur.msg.event == EVENT_NONE) {
        execute_action();
        return;
    }

    State* next = dispatch();
    if(next != cur.s) {
        State* lca = find_lca(cur.s, next);
        exit_up_to(cur.s, lca);
        enter_down_to(lca, next);
        cur.s = next;
        cur.msg.event = EVENT_NONE;
    }

    execute_action();
}

// ! ========================= 状 态 机 实 现 ========================= ! //

static State* dispatch(void) {
    while(cur.s) {
        if(cur.s->handle) {
            State* next = cur.s->handle();
            if(next) return next;
        }
        cur.s = cur.s->_parent_;
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
    int diff = (int)fabs((double)(depth1 - depth2));

    while(--diff) { deeper = deeper->_parent_; }
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

    while(s && s != from) {
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

// ! ========================= 状 态 处 理 函 数 实 现 ========================= ! //

static State* handle_init(void) {
    switch(cur.msg.event) {
        case EVENT_START_SEARCH:
            return &state_searching;
        default:
            return NULL;
    }
}
static State* handle_normal(void) {
    switch(cur.msg.event) {
        case EVENT_START_SEARCH:
            return &state_searching;
        default:
            return NULL;
    }
}
static State* handle_idle(void) {
    switch(cur.msg.event) {
        default:
            return NULL;
    }
}
static State* handle_searching(void) {
    switch(cur.msg.event) {
        default:
            return NULL;
    }
}
static State* handle_ik_pose(void) {
    switch(cur.msg.event) {
        default:
            return NULL;
    }
}
static State* handle_moving(void) {
    switch(cur.msg.event) {
        default:
            return NULL;
    }
}
static State* handle_lasering(void) {
    switch(cur.msg.event) {
        default:
            return NULL;
    }
}
static State* handle_error(void) {
    switch(cur.msg.event) {
        default:
            return NULL;
    }
}

// ! ========================= 状 态 进 入 函 数 实 现 ========================= ! //

static void entry_init(void) {
    dm.enable(0x01);
    dm.enable(0x02);
    dm.enable(0x03);
    dm.enable(0x04);
    dm.enable(0x05);
    dm.enable(0x06);

    fsm_data.info.ssid = "K230";
    fsm_data.info.password = "12345678";
    fsm_data.info.protocol = UDP;
    fsm_data.info.role = Client;
    strcpy(fsm_data.info.ip, "192.168.169.1");
    fsm_data.info.remote_port = 8080;
    fsm_data.info.local_port = 5000;
    if(wifi.join_ap(fsm_data.info.ssid, fsm_data.info.password) != wifi.OK) printf("WiFi 连接失败!\r\n");
    else printf("WiFi 连接成功!\r\n");
}
static void entry_normal(void) {}
static void entry_idle(void) {}
static void entry_searching(void) {}
static void entry_ik_pose(void) {}
static void entry_moving(void) {}
static void entry_lasering(void) {}
static void entry_error(void) {}

// ! ========================= 状 态 退 出 函 数 实 现 ========================= ! //

static void exit_init(void) {}
static void exit_normal(void) {}
static void exit_idle(void) {}
static void exit_searching(void) {}
static void exit_ik_pose(void) {}
static void exit_moving(void) {}
static void exit_lasering(void) {}
static void exit_error(void) {}

// ! ========================= 状 态 持 续 函 数 实 现 ========================= ! //

static void action_init(void) {}
static void action_normal(void) {}
static void action_idle(void) {}
static void action_searching(void) {}
static void action_ik_pose(void) {}
static void action_moving(void) {}
static void action_lasering(void) {}
static void action_error(void) {}
