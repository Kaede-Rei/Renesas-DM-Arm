#include "app/fsm.h"

#include <stddef.h>

// ! ========================= 变 量 声 明 ========================= ! //

Event current_event = EVENT_NONE;
State* current_state = &state_idle;

#define SX(name, parent, entry, exit, action) \
    static State* handle_##name(Event event);
STATE_TABLE
#undef SX

static void searching_entry(void);
static void error_entry(void);
static void laser_entry(void);

static void laser_exit(void);

static void normal_action(void);
static void idle_action(void);
static void searching_action(void);
static void ik_pose_action(void);
static void moving_action(void);
static void lasering_action(void);

#define SX(n, p, en, ex, a) \
    State state_##n = { \
        .name = #n, \
        .handle = handle_##n, \
        .entry = en, \
        .exit = ex, \
        .action = a, \
        ._parent_ = p \
    };
STATE_TABLE
#undef SX

// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //



// ! ========================= 私 有 函 数 实 现 ========================= ! //

static State* handle_normal(Event event) {
    switch(event) {
        case EVENT_START_SEARCH:
            return &state_searching;
        default:
            return NULL;
    }
}
static State* handle_idle(Event event) {
    switch(event) {
        default:
            return NULL;
    }
}
static State* handle_searching(Event event) {
    switch(event) {
        default:
            return NULL;
    }
}
static State* handle_ik_pose(Event event) {
    switch(event) {
        default:
            return NULL;
    }
}
static State* handle_moving(Event event) {
    switch(event) {
        default:
            return NULL;
    }
}
static State* handle_lasering(Event event) {
    switch(event) {
        default:
            return NULL;
    }
}
static State* handle_error(Event event) {
    switch(event) {
        default:
            return NULL;
    }
}

static void searching_entry(void) {}
static void error_entry(void) {}
static void laser_entry(void) {}

static void laser_exit(void) {}

static void normal_action(void) {}
static void idle_action(void) {}
static void searching_action(void) {}
static void ik_pose_action(void) {}
static void moving_action(void) {}
static void lasering_action(void) {}
