#ifndef _hfsm_h_
#define _hfsm_h_

#include "hfsm_config.h"
#include "hfsm_core.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

#define hfsm hfsm_api_instance

#define HFSM_STATUS_TABLE \
    SX(OK) \
    SX(INVALID_ARG) \
    SX(NOT_INIT) \
    SX(NO_INITIAL_STATE) \
    SX(STARTED) \
    SX(NOT_STARTED) \
    SX(NO_SPACE)

#define SX(name) HFSM_STATUS_##name,
typedef enum {
    HFSM_STATUS_TABLE
} HfsmStatus;
#undef SX

typedef struct {
    HfsmMachine machine;
    const HfsmState* initial_state;
    HfsmState states[HFSM_MAX_STATES];
    uint16_t state_count;
    bool initialized;
    bool started;
} Hfsm;

#define SX(name) const HfsmStatus name;
extern const struct HfsmApi {
    struct {
        HFSM_STATUS_TABLE
    };

    HfsmStatus(*init)(Hfsm* fsm, void* context);
    HfsmStatus(*set_context)(Hfsm* fsm, void* context);
    HfsmStatus(*set_initial)(Hfsm* fsm, const HfsmState* initial_state);
    HfsmState* (*add_state)(Hfsm* fsm, const char* name);
    HfsmState* (*add_substate)(Hfsm* fsm, HfsmState* parent, const char* name);

    HfsmState* (*set_parent)(HfsmState* s, const HfsmState* parent);
    HfsmState* (*set_handle)(HfsmState* s, HfsmHandleFn handle);
    HfsmState* (*set_entry)(HfsmState* s, HfsmHookFn entry);
    HfsmState* (*set_exit)(HfsmState* s, HfsmHookFn exit);
    HfsmState* (*set_action)(HfsmState* s, HfsmHookFn action);
    HfsmState* (*set_user_data)(HfsmState* s, void* user_data);

    HfsmStatus(*start)(Hfsm* fsm);
    HfsmStatus(*pause)(Hfsm* fsm);
    HfsmStatus(*go_on)(Hfsm* fsm);
    HfsmStatus(*reset)(Hfsm* fsm);

    HfsmStatus(*post)(Hfsm* fsm, HfsmEventId event_id, const void* data);
    HfsmStatus(*clear)(Hfsm* fsm);
    HfsmStatus(*process)(Hfsm* fsm);
    HfsmStatus(*process_all)(Hfsm* fsm);

    struct {
        HfsmResult(*ignore)(void);
        HfsmResult(*handled)(void);
        HfsmResult(*transition)(const HfsmState* next_state);
    } res;

    const HfsmState* (*state)(const Hfsm* fsm);
    const HfsmState* (*dispatching)(const Hfsm* fsm);
    void* (*context)(Hfsm* fsm);
    const void* (*const_context)(const Hfsm* fsm);
} hfsm_api_instance;
#undef SX

// ! ========================= 接 口 函 数 声 明 ========================= ! //

HfsmStatus hfsm_init(Hfsm* fsm, void* context);
HfsmStatus hfsm_set_context(Hfsm* fsm, void* context);
HfsmStatus hfsm_set_initial(Hfsm* fsm, const HfsmState* initial_state);
HfsmState* hfsm_add_state(Hfsm* fsm, const char* name);
HfsmState* hfsm_add_substate(Hfsm* fsm, HfsmState* parent, const char* name);

HfsmState* hfsm_set_parent(HfsmState* s, const HfsmState* parent);
HfsmState* hfsm_set_handle(HfsmState* s, HfsmHandleFn handle);
HfsmState* hfsm_set_entry(HfsmState* s, HfsmHookFn entry);
HfsmState* hfsm_set_exit(HfsmState* s, HfsmHookFn exit);
HfsmState* hfsm_set_action(HfsmState* s, HfsmHookFn action);
HfsmState* hfsm_set_user_data(HfsmState* s, void* user_data);

HfsmStatus hfsm_start(Hfsm* fsm);
HfsmStatus hfsm_pause(Hfsm* fsm);
HfsmStatus hfsm_go_on(Hfsm* fsm);
HfsmStatus hfsm_reset(Hfsm* fsm);

HfsmStatus hfsm_post(Hfsm* fsm, HfsmEventId event_id, const void* data);
HfsmStatus hfsm_clear(Hfsm* fsm);
HfsmStatus hfsm_process(Hfsm* fsm);
HfsmStatus hfsm_process_all(Hfsm* fsm);


HfsmResult hfsm_ignore(void);
HfsmResult hfsm_handled(void);
HfsmResult hfsm_transition(const HfsmState* next_state);

const HfsmState* hfsm_state(const Hfsm* fsm);
const HfsmState* hfsm_dispatching(const Hfsm* fsm);
void* hfsm_context(Hfsm* fsm);
const void* hfsm_const_context(const Hfsm* fsm);

#endif
