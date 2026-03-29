#include "hfsm.h"
#include "hfsm_core.h"

#include <stddef.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define hfsm hfsm_api_instance

#define SX(name) .name = HFSM_STATUS_##name,
const struct HfsmApi hfsm_api_instance = {
    {
        HFSM_STATUS_TABLE
    },
    .init = hfsm_init,
    .set_context = hfsm_set_context,
    .set_initial = hfsm_set_initial,
    .add_state = hfsm_add_state,
    .add_substate = hfsm_add_substate,
    .set_parent = hfsm_set_parent,
    .set_handle = hfsm_set_handle,
    .set_entry = hfsm_set_entry,
    .set_exit = hfsm_set_exit,
    .set_action = hfsm_set_action,
    .set_user_data = hfsm_set_user_data,
    .start = hfsm_start,
    .pause = hfsm_pause,
    .go_on = hfsm_go_on,
    .reset = hfsm_reset,
    .post = hfsm_post,
    .clear = hfsm_clear,
    .process = hfsm_process,
    .process_all = hfsm_process_all,
    .res = {
        .ignore = hfsm_ignore,
        .handled = hfsm_handled,
        .transition = hfsm_transition
    },
    .state = hfsm_state,
    .dispatching = hfsm_dispatching,
    .context = hfsm_context,
    .const_context = hfsm_const_context
};
#undef SX

// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //

HfsmStatus hfsm_init(Hfsm* fsm, void* context) {
    if(fsm == NULL) return hfsm.INVALID_ARG;

    memset(fsm, 0, sizeof(Hfsm));
    fsm->machine.context = context;
    fsm->started = false;
    fsm->initialized = true;

    return hfsm.OK;
}

HfsmStatus hfsm_set_context(Hfsm* fsm, void* context) {
    if(fsm == NULL) return hfsm.INVALID_ARG;
    if(fsm->initialized == false) return hfsm.NOT_INIT;
    if(fsm->started) return hfsm.STARTED;

    fsm->machine.context = context;

    return hfsm.OK;
}

HfsmStatus hfsm_set_initial(Hfsm* fsm, const HfsmState* initial_state) {
    if(fsm == NULL) return hfsm.INVALID_ARG;
    if(fsm->initialized == false) return hfsm.NOT_INIT;
    if(initial_state == NULL) return hfsm.INVALID_ARG;
    if(fsm->started) return hfsm.STARTED;

    fsm->initial_state = initial_state;

    return hfsm.OK;
}

HfsmState* hfsm_add_state(Hfsm* fsm, const char* name) {
    if(fsm == NULL || name == NULL) return NULL;
    if(fsm->initialized == false) return NULL;
    if(fsm->state_count >= HFSM_MAX_STATES) return NULL;
    if(fsm->started) return NULL;

    HfsmState* s = &fsm->states[fsm->state_count++];
    memset(s, 0, sizeof(HfsmState));
    s->name = name;

    return s;
}

HfsmState* hfsm_add_substate(Hfsm* fsm, HfsmState* parent, const char* name) {
    if(parent == NULL) return NULL;

    HfsmState* s = hfsm_add_state(fsm, name);
    if(s == NULL) return NULL;

    s->parent = parent;

    return s;
}

HfsmState* hfsm_set_parent(HfsmState* s, const HfsmState* parent) {
    if(s == NULL) return NULL;
    s->parent = parent;

    return s;
}

HfsmState* hfsm_set_handle(HfsmState* s, HfsmHandleFn handle) {
    if(s == NULL) return NULL;
    s->handle = handle;

    return s;
}

HfsmState* hfsm_set_entry(HfsmState* s, HfsmHookFn entry) {
    if(s == NULL) return NULL;
    s->entry = entry;

    return s;
}

HfsmState* hfsm_set_exit(HfsmState* s, HfsmHookFn exit) {
    if(s == NULL) return NULL;
    s->exit = exit;

    return s;
}

HfsmState* hfsm_set_action(HfsmState* s, HfsmHookFn action) {
    if(s == NULL) return NULL;
    s->action = action;

    return s;
}

HfsmState* hfsm_set_user_data(HfsmState* s, void* user_data) {
    if(s == NULL) return NULL;
    s->user_data = user_data;

    return s;
}

HfsmStatus hfsm_start(Hfsm* fsm) {
    if(fsm == NULL) return hfsm.INVALID_ARG;
    if(fsm->initialized == false) return hfsm.NOT_INIT;
    if(fsm->initial_state == NULL) return hfsm.NO_INITIAL_STATE;
    if(fsm->started) return hfsm.STARTED;

    hfsm_core.init(&fsm->machine, fsm->initial_state, fsm->machine.context);
    fsm->started = true;

    return hfsm.OK;
}

HfsmStatus hfsm_pause(Hfsm* fsm) {
    if(fsm == NULL) return hfsm.INVALID_ARG;
    if(fsm->initialized == false) return hfsm.NOT_INIT;
    if(fsm->started == false) return hfsm.NOT_STARTED;

    fsm->started = false;

    return hfsm.OK;
}

HfsmStatus hfsm_go_on(Hfsm* fsm) {
    if(fsm == NULL) return hfsm.INVALID_ARG;
    if(fsm->initialized == false) return hfsm.NOT_INIT;
    if(fsm->started) return hfsm.STARTED;

    fsm->started = true;

    return hfsm.OK;
}

HfsmStatus hfsm_reset(Hfsm* fsm) {
    if(fsm == NULL) return hfsm.INVALID_ARG;
    if(fsm->initialized == false) return hfsm.NOT_INIT;

    void* context = fsm->machine.context;
    hfsm_core.init(&fsm->machine, fsm->initial_state, context);
    fsm->started = true;

    return hfsm.OK;
}

HfsmStatus hfsm_post(Hfsm* fsm, HfsmEventId event_id, const void* data) {
    if(fsm == NULL) return hfsm.INVALID_ARG;
    if(fsm->initialized == false) return hfsm.NOT_INIT;
    if(fsm->started == false) return hfsm.NOT_STARTED;
    if(event_id == HFSM_EVENT_NONE) return hfsm.INVALID_ARG;

    return hfsm_core.post(&fsm->machine, event_id, data) ? hfsm.OK : hfsm.NO_SPACE;
}

HfsmStatus hfsm_clear(Hfsm* fsm) {
    if(fsm == NULL) return hfsm.INVALID_ARG;
    if(fsm->initialized == false) return hfsm.NOT_INIT;
    if(fsm->started == false) return hfsm.NOT_STARTED;

    hfsm_core.clear(&fsm->machine);

    return hfsm.OK;
}

HfsmStatus hfsm_process(Hfsm* fsm) {
    if(fsm == NULL) return hfsm.INVALID_ARG;
    if(fsm->initialized == false) return hfsm.NOT_INIT;
    if(fsm->started == false) return hfsm.NOT_STARTED;

    hfsm_core.process(&fsm->machine);

    return hfsm.OK;
}

HfsmStatus hfsm_process_all(Hfsm* fsm) {
    if(fsm == NULL) return hfsm.INVALID_ARG;
    if(fsm->initialized == false) return hfsm.NOT_INIT;
    if(fsm->started == false) return hfsm.NOT_STARTED;

    hfsm_core.process_all(&fsm->machine);

    return hfsm.OK;
}

HfsmResult hfsm_ignore(void) {
    HfsmResult res = {
        .type = hfsm_core.res.IGNORE,
        .next_state = NULL
    };

    return res;
}

HfsmResult hfsm_handled(void) {
    HfsmResult res = {
        .type = hfsm_core.res.HANDLED,
        .next_state = NULL
    };

    return res;
}

HfsmResult hfsm_transition(const HfsmState* next_state) {
    HfsmResult res = {
        .type = hfsm_core.res.TRANSITION,
        .next_state = next_state
    };

    return res;
}

const HfsmState* hfsm_state(const Hfsm* fsm) {
    if(fsm == NULL) return NULL;
    return hfsm_core.state(&fsm->machine);
}

const HfsmState* hfsm_dispatching(const Hfsm* fsm) {
    if(fsm == NULL) return NULL;
    return hfsm_core.dispatching(&fsm->machine);
}

void* hfsm_context(Hfsm* fsm) {
    if(fsm == NULL) return NULL;
    return hfsm_core.context(&fsm->machine);
}

const void* hfsm_const_context(const Hfsm* fsm) {
    if(fsm == NULL) return NULL;
    return hfsm_core.const_context(&fsm->machine);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //


