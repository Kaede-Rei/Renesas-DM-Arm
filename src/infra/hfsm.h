#ifndef _hfsm_h_
#define _hfsm_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

#define hfsm hfsm_instance

typedef uint16_t HfsmEventId;

enum { HFSM_EVENT_NONE = 0 };

typedef struct {
    HfsmEventId id;
    void* data;
} HfsmEvent;

typedef struct HfsmMachine HfsmMachine;
typedef struct HfsmState HfsmState;

typedef const HfsmState* (*HfsmHandleFn)(HfsmMachine* m, const HfsmEvent* e);
typedef void(*HfsmHookFn)(HfsmMachine* m);

struct HfsmState {
    const char* name;
    const HfsmState* parent;

    HfsmHandleFn handle;
    HfsmHookFn entry;
    HfsmHookFn exit;
    HfsmHookFn action;
};

struct HfsmMachine {
    const HfsmState* current_state;
    HfsmEvent pending_event;
    void* context;
};

extern const struct HfsmInstance {
    void(*init)(HfsmMachine* m, const HfsmState* initial_state, void* context);
    bool(*post)(HfsmMachine* m, HfsmEventId event_id, void* data);
    void(*clear)(HfsmMachine* m);
    void(*process)(HfsmMachine* m);
    const HfsmState* (*current)(const HfsmMachine* m);
    void* (*context)(const HfsmMachine* m);
    bool(*is_descendant_of)(const HfsmState* state, const HfsmState* ancestor);
    bool(*transition)(HfsmMachine* m, const HfsmState* target_state);
    bool(*has_pending)(const HfsmMachine* m);
} hfsm_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void hfsm_init(HfsmMachine* m, const HfsmState* initial_state, void* context);
bool hfsm_post(HfsmMachine* m, HfsmEventId event_id, void* data);
void hfsm_clear(HfsmMachine* m);
void hfsm_process(HfsmMachine* m);
const HfsmState* hfsm_get_current_state(const HfsmMachine* m);
void* hfsm_get_context(const HfsmMachine* m);
bool hfsm_is_descendant_of(const HfsmState* state, const HfsmState* ancestor);
bool hfsm_transition(HfsmMachine* m, const HfsmState* target_state);
bool hfsm_has_pending_event(const HfsmMachine* m);

#endif
