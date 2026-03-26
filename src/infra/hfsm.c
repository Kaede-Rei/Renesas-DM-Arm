#include "hfsm.h"

#include <stddef.h>
#include <stdlib.h>

#ifndef HFSM_DEPTH
#define HFSM_DEPTH 8
#endif

// ! ========================= 变 量 声 明 ========================= ! //

const struct HfsmInstance hfsm_instance = {
    .init = hfsm_init,
    .post = hfsm_post,
    .clear = hfsm_clear,
    .process = hfsm_process,
    .current = hfsm_get_current_state,
    .is_descendant_of = hfsm_is_descendant_of,
    .transition = hfsm_transition,
    .has_pending = hfsm_has_pending_event
};

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static const HfsmState* dispatch(HfsmMachine* m, const HfsmEvent* e);
static const HfsmState* find_lca(const HfsmState* s1, const HfsmState* s2);
static void exit_up_to(const HfsmState* from, const HfsmState* to, HfsmMachine* m);
static void enter_down_to(const HfsmState* from, const HfsmState* to, HfsmMachine* m);
static void execute_action(HfsmMachine* m);
static bool event_filter(HfsmMachine* m, HfsmEventId event_id);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void hfsm_init(HfsmMachine* m, const HfsmState* initial_state, void* context) {
    if(m == NULL) return;

    if(initial_state == NULL) return;

    m->current_state = initial_state;
    m->pending_event.id = HFSM_EVENT_NONE;
    m->pending_event.data = NULL;
    m->context = context;

    enter_down_to(NULL, initial_state, m);
}

bool hfsm_post(HfsmMachine* m, HfsmEventId event_id, void* data) {
    if(m == NULL || event_id == HFSM_EVENT_NONE) return false;
    if(m->pending_event.id != HFSM_EVENT_NONE) return false;
    if(event_filter(m, event_id) == false) return false;

    m->pending_event.id = event_id;
    m->pending_event.data = data;

    return true;
}

void hfsm_clear(HfsmMachine* m) {
    if(m == NULL) return;

    m->pending_event.id = HFSM_EVENT_NONE;
    m->pending_event.data = NULL;
}

void hfsm_process(HfsmMachine* m) {
    if(m == NULL || m->current_state == NULL) return;

    const HfsmState* current_state = m->current_state;
    if(m->pending_event.id != HFSM_EVENT_NONE) {
        const HfsmState* next_state = dispatch(m, &m->pending_event);

        if(next_state != NULL && next_state != current_state) {
            const HfsmState* lca = find_lca(current_state, next_state);
            exit_up_to(current_state, lca, m);
            enter_down_to(lca, next_state, m);
            m->current_state = next_state;
        }

        hfsm_clear(m);
    }

    if(m->current_state != NULL && m->current_state->action != NULL) {
        execute_action(m);
    }
}

const HfsmState* hfsm_get_current_state(const HfsmMachine* m) {
    if(m == NULL) return NULL;

    return m->current_state;
}

void* hfsm_get_context(const HfsmMachine* m) {
    if(m == NULL) return NULL;

    return m->context;
}

bool hfsm_is_descendant_of(const HfsmState* state, const HfsmState* ancestor) {
    if(state == NULL || ancestor == NULL) return false;

    const HfsmState* current = state;
    while(current != NULL) {
        if(current == ancestor) return true;
        current = current->parent;
    }

    return false;
}

bool hfsm_transition(HfsmMachine* m, const HfsmState* target_state) {
    if(m == NULL || m->current_state == NULL || target_state == NULL) return false;

    const HfsmState* current_state = m->current_state;
    if(current_state == target_state) return true;

    const HfsmState* lca = find_lca(current_state, target_state);
    if(lca == NULL) return false;

    exit_up_to(current_state, lca, m);
    enter_down_to(lca, target_state, m);

    m->current_state = target_state;

    return true;
}

bool hfsm_has_pending_event(const HfsmMachine* m) {
    if(m == NULL) return false;

    return m->pending_event.id != HFSM_EVENT_NONE;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static const HfsmState* dispatch(HfsmMachine* m, const HfsmEvent* e) {
    if(m == NULL || e == NULL || m->current_state == NULL) return NULL;

    const HfsmState* state = m->current_state;
    while(state != NULL) {
        if(state->handle != NULL) {
            const HfsmState* next_state = state->handle(m, e);
            if(next_state != NULL) {
                return next_state;
            }
        }
        state = state->parent;
    }

    return m->current_state;
}

static const HfsmState* find_lca(const HfsmState* s1, const HfsmState* s2) {
    if(!s1 || !s2) return NULL;

    int depth1 = 0, depth2 = 0;

    const HfsmState* s = s1;
    while(s) { depth1++; s = s->parent; }
    s = s2;
    while(s) { depth2++; s = s->parent; }

    const HfsmState* deeper = depth1 > depth2 ? s1 : s2;
    const HfsmState* shallower = depth1 > depth2 ? s2 : s1;
    int diff = abs(depth1 - depth2);

    while(diff--) { deeper = deeper->parent; }
    while(deeper != shallower) {
        deeper = deeper->parent;
        shallower = shallower->parent;
    }

    return deeper;
}

static void exit_up_to(const HfsmState* from, const HfsmState* to, HfsmMachine* m) {
    const HfsmState* s = from;
    while(s && s != to) {
        if(s->exit) s->exit(m);
        s = s->parent;
    }
}

static void enter_down_to(const HfsmState* from, const HfsmState* to, HfsmMachine* m) {
    const HfsmState* path[HFSM_DEPTH];
    int depth = 0;
    const HfsmState* s = to;

    while(s && s != from && depth < HFSM_DEPTH) {
        path[depth++] = s;
        s = s->parent;
    }

    while(depth--) {
        s = path[depth];
        if(s->entry) s->entry(m);
    }
}

static void execute_action(HfsmMachine* m) {
    const HfsmState* s = m->current_state;
    while(s) {
        if(s->action) s->action(m);
        s = s->parent;
    }
}

static bool event_filter(HfsmMachine* m, HfsmEventId event_id) {
    const HfsmState* s = m->current_state;
    while(s) {
        if(s->handle) {
            HfsmEventId old_event = m->pending_event.id;
            m->pending_event.id = event_id;

            const HfsmState* next = s->handle(m, &m->pending_event);

            m->pending_event.id = old_event;

            if(next) return true;
        }
        s = s->parent;
    }

    return false;
}
