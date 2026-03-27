#include "hfsm.h"
#include "hfsm_config.h"

#include <assert.h>
#include <stddef.h>
#include <stdlib.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 全局 HFSM 实例，提供接口函数和资源函数的访问
 * @param init 初始化状态机实例
 * @param post 向状态机实例发送事件
 * @param clear 清除状态机实例的待处理事件
 * @param process 处理状态机实例的待处理事件
 * @param state 获取状态机实例的当前状态
 * @param context 获取状态机实例的上下文指针
 * @param const_context 获取状态机实例的上下文指针（只读）
 * @param is_descendant_of 判断一个状态是否是另一个状态的子状态
 * @param transition 从当前状态转换到目标状态
 * @param has_pending 判断状态机实例是否有待处理事件
 * @param res 资源函数结构体，包含 ignore、handled 和 transition 三个函数
 *      - ignore 返回一个表示事件被忽略的结果
 *      - handled 返回一个表示事件已处理但不进行状态转换的结果
 *      - transition 返回一个表示事件已处理并进行状态转换的结果，next_state 字段指向目标状态
 */
#define RX(name) .name = HFSM_RES_##name,
const struct HfsmInstance hfsm_instance = {
    .init = hfsm_init,
    .post = hfsm_post,
    .clear = hfsm_clear,
    .process = hfsm_process,
    .state = hfsm_get_current_state,
    .context = hfsm_get_context,
    .const_context = hfsm_get_const_context,
    .is_descendant_of = hfsm_is_descendant_of,
    .transition = hfsm_transition,
    .has_pending = hfsm_has_pending_event,
    .res = {
        .ignore = hfsm_res_ignore,
        .handled = hfsm_res_handled,
        .transition = hfsm_res_transition,
        HFSM_RES_TYPE
    }
};
#undef RX
#define hfsm hfsm_instance

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static HfsmResult dispatch(HfsmMachine* m, const HfsmEvent* e);
static const HfsmState* find_lca(const HfsmState* s1, const HfsmState* s2);
static void exit_up_to(const HfsmState* from, const HfsmState* to, HfsmMachine* m);
static void enter_down_to(const HfsmState* from, const HfsmState* to, HfsmMachine* m);
static void execute_action(HfsmMachine* m);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化状态机实例
 * @param m 状态机实例指针
 * @param initial_state 初始状态指针
 * @param context 用户自定义上下文指针
 */
void hfsm_init(HfsmMachine* m, const HfsmState* initial_state, void* context) {
    if(m == NULL) return;

    m->current_state = NULL;
    m->pending_event.id = HFSM_EVENT_NONE;
    m->pending_event.data = NULL;
    m->context = context;

    if(initial_state == NULL) return;

    m->current_state = initial_state;
    enter_down_to(NULL, initial_state, m);
}

/**
 * @brief 向状态机实例发送事件
 * @param m 状态机实例指针
 * @param event_id 事件ID
 * @param data 事件数据指针
 * @return true 表示事件发送成功，false 表示失败
 */
bool hfsm_post(HfsmMachine* m, HfsmEventId event_id, void* data) {
    if(m == NULL || event_id == HFSM_EVENT_NONE) return false;
    if(m->pending_event.id != HFSM_EVENT_NONE) return false;

    m->pending_event.id = event_id;
    m->pending_event.data = data;

    return true;
}

/**
 * @brief 获取状态机实例的当前状态
 * @param m 状态机实例指针
 * @return 当前状态指针，若 m 为 NULL 则返回 NULL
 */
void hfsm_clear(HfsmMachine* m) {
    if(m == NULL) return;

    m->pending_event.id = HFSM_EVENT_NONE;
    m->pending_event.data = NULL;
}

/**
 * @brief 处理状态机实例的待处理事件
 * @param m 状态机实例指针
 */
void hfsm_process(HfsmMachine* m) {
    if(m == NULL || m->current_state == NULL) return;

    const HfsmState* current_state = m->current_state;
    if(m->pending_event.id != HFSM_EVENT_NONE) {
        HfsmResult result = dispatch(m, &m->pending_event);

        if(result.type == HFSM_RES_TRANSITION && result.next_state != NULL && result.next_state != current_state) {
            const HfsmState* lca = find_lca(current_state, result.next_state);
            exit_up_to(current_state, lca, m);
            enter_down_to(lca, result.next_state, m);
            m->current_state = result.next_state;
        }

        hfsm_clear(m);
    }

    if(m->current_state == NULL) return;

#if HFSM_RUN_PARENT_ACTIONS
    execute_action(m);
#else
    if(m->current_state->action) m->current_state->action(m);
#endif
}

/**
 * @brief 获取状态机实例的当前状态
 * @param m 状态机实例指针
 * @return 当前状态指针，若 m 为 NULL 则返回 NULL
 */
const HfsmState* hfsm_get_current_state(const HfsmMachine* m) {
    if(m == NULL) return NULL;

    return m->current_state;
}

/**
 * @brief 获取状态机实例的上下文指针
 * @param m 状态机实例指针
 * @return 上下文指针，若 m 为 NULL 则返回 NULL
 */
void* hfsm_get_context(HfsmMachine* m) {
    if(m == NULL) return NULL;

    return m->context;
}

/**
 * @brief 获取状态机实例的上下文指针（只读）
 * @param m 状态机实例指针
 * @return 上下文指针，若 m 为 NULL 则返回 NULL
 */
const void* hfsm_get_const_context(const HfsmMachine* m) {
    if(m == NULL) return NULL;

    return m->context;
}

/**
 * @brief 判断一个状态是否是另一个状态的子状态
 * @param state 待判断状态指针
 * @param ancestor 祖先状态指针
 * @return 若 state 是 ancestor 的子状态返回 true，否则返回 false
 */
bool hfsm_is_descendant_of(const HfsmState* state, const HfsmState* ancestor) {
    if(state == NULL || ancestor == NULL) return false;

    const HfsmState* current = state;
    while(current != NULL) {
        if(current == ancestor) return true;
        current = current->parent;
    }

    return false;
}

/**
 * @brief 从当前状态转换到目标状态
 * @param m 状态机实例指针
 * @param target_state 目标状态指针
 * @return 转换成功返回 true，失败返回 false
 */
bool hfsm_transition(HfsmMachine* m, const HfsmState* target_state) {
    if(m == NULL || m->current_state == NULL || target_state == NULL) return false;

    const HfsmState* current_state = m->current_state;
    if(current_state == target_state) return true;

    const HfsmState* lca = find_lca(current_state, target_state);

    exit_up_to(current_state, lca, m);
    enter_down_to(lca, target_state, m);

    m->current_state = target_state;

    return true;
}

/**
 * @brief 判断状态机实例是否有待处理事件
 * @param m 状态机实例指针
 * @return true 表示有待处理事件，false 表示没有
 */
bool hfsm_has_pending_event(const HfsmMachine* m) {
    if(m == NULL) return false;

    return m->pending_event.id != HFSM_EVENT_NONE;
}

/**
 * @brief 返回一个表示事件被忽略的结果
 * @return HfsmResult 结果类型为 IGNORE，next_state 字段为 NULL
 */
HfsmResult hfsm_res_ignore(void) {
    return (HfsmResult) { .type = hfsm.res.IGNORE, .next_state = NULL };
}

/**
 * @brief 返回一个表示事件已处理但不进行状态转换的结果
 * @return HfsmResult 结果类型为 HANDLED，next_state 字段为 NULL
 */
HfsmResult hfsm_res_handled(void) {
    return (HfsmResult) { .type = hfsm.res.HANDLED, .next_state = NULL };
}

/**
 * @brief 返回一个表示事件已处理并进行状态转换的结果
 * @param next_state 目标状态指针
 * @return HfsmResult 结果类型为 TRANSITION，next_state 字段指向目标状态
 */
HfsmResult hfsm_res_transition(const HfsmState* next_state) {
    return (HfsmResult) { .type = hfsm.res.TRANSITION, .next_state = next_state };
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 分发事件到状态机实例的当前状态及其祖先状态
 * @param m 状态机实例指针
 * @param e 事件指针
 * @return HfsmResult 处理结果，可能是 IGNORE、HANDLED 或 TRANSITION
 */
static HfsmResult dispatch(HfsmMachine* m, const HfsmEvent* e) {
    if(m == NULL || e == NULL || m->current_state == NULL) return hfsm.res.ignore();

    const HfsmState* state = m->current_state;
    while(state != NULL) {
        if(state->handle != NULL) {
            HfsmResult result = state->handle(m, e);
            if(result.type == HFSM_RES_HANDLED) return result;
            if(result.type == HFSM_RES_TRANSITION && result.next_state != NULL) return result;
        }
        state = state->parent;
    }

    return hfsm.res.ignore();
}

/**
 * @brief 查找两个状态的最近公共祖先
 * @param s1 状态指针1
 * @param s2 状态指针2
 * @return 最近公共祖先状态指针，若没有公共祖先则返回 NULL
 */
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

/**
 * @brief 从状态 from 退出到状态 to（不包括 to），调用每个状态的 exit 钩子函数
 * @param from 起始状态指针
 * @param to 目标状态指针
 * @param m 状态机实例指针
 */
static void exit_up_to(const HfsmState* from, const HfsmState* to, HfsmMachine* m) {
    const HfsmState* s = from;
    while(s && s != to) {
        if(s->exit) s->exit(m);
        s = s->parent;
    }
}

/**
 * @brief 从状态 from 进入到状态 to（不包括 from），调用每个状态的 entry 钩子函数
 * @param from 起始状态指针
 * @param to 目标状态指针
 * @param m 状态机实例指针
 */
static void enter_down_to(const HfsmState* from, const HfsmState* to, HfsmMachine* m) {
    const HfsmState* path[HFSM_DEPTH];
    int depth = 0;
    const HfsmState* s = to;

    while(s && s != from) {
#if HFSM_ENABLE_ASSERT
        assert(depth < HFSM_DEPTH);
#else
        if(depth >= HFSM_DEPTH) return;
#endif
        path[depth++] = s;
        s = s->parent;
    }

    while(depth--) {
        s = path[depth];
        if(s->entry) s->entry(m);
    }
}

/**
 * @brief 执行当前状态及其祖先状态的 action 函数
 * @param m 状态机实例指针
 */
static void execute_action(HfsmMachine* m) {
    const HfsmState* s = m->current_state;
    while(s) {
        if(s->action) s->action(m);
        s = s->parent;
    }
}
