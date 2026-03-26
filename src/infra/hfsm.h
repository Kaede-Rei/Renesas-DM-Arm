#ifndef _hfsm_h_
#define _hfsm_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 用户自定义名称
 */
#define hfsm hfsm_instance

/**
 * @brief 事件标识符类型
 */
typedef uint16_t HfsmEventId;

/**
 * @brief 无事件枚举
 */
enum { HFSM_EVENT_NONE = 0 };

/**
 * @brief 事件结构体
 */
typedef struct {
    HfsmEventId id;
    void* data;
} HfsmEvent;

/**
 * @brief 状态机实例结构体
 */
typedef struct HfsmMachine HfsmMachine;

/**
 * @brief 状态机状态结构体
 */
typedef struct HfsmState HfsmState;

/**
 * @brief 状态机处理结果类型枚举
 * @param IGNORE 忽略事件
 * @param HANDLED 事件已处理但不进行状态转换
 * @param TRANSITION 事件已处理并进行状态转换，next_state 字段指向目标状态
 */
typedef enum {
    HFSM_RES_IGNORE,
    HFSM_RES_HANDLED,
    HFSM_RES_TRANSITION
} HfsmResType;

/**
 * @brief 状态机处理结果结构体
 * @param type 处理结果类型
 * @param next_state 目标状态指针，仅当 type 为 HFSM_RES_TRANSITION 时有效
 */
typedef struct {
    HfsmResType type;
    const HfsmState* next_state;
} HfsmResult;

/**
 * @brief 状态机处理函数类型
 * @param m 状态机实例指针
 * @param e 事件指针
 */
typedef HfsmResult(*HfsmHandleFn)(HfsmMachine* m, const HfsmEvent* e);

/**
 * @brief 状态机钩子函数类型
 * @param m 状态机实例指针
 */
typedef void(*HfsmHookFn)(HfsmMachine* m);

/**
 * @brief 状态机状态结构体
 * @param name 状态名称
 * @param parent 父状态指针
 * @param handle 事件处理函数指针
 * @param entry 进入钩子函数指针
 * @param exit 退出钩子函数指针
 * @param action 内部动作函数指针
 */
struct HfsmState {
    const char* name;
    const HfsmState* parent;

    HfsmHandleFn handle;
    HfsmHookFn entry;
    HfsmHookFn exit;
    HfsmHookFn action;
};

/**
 * @brief 状态机实例结构体
 * @param current_state 当前状态指针
 * @param pending_event 待处理事件
 * @param context 用户自定义上下文指针
 */
struct HfsmMachine {
    const HfsmState* current_state;
    HfsmEvent pending_event;
    void* context;
};

/**
 * @brief 状态机实例单例，包含所有状态机相关的接口函数指针
 */
extern const struct HfsmInstance {
    /**
     * @brief 初始化状态机实例
     * @param m 状态机实例指针
     * @param initial_state 初始状态指针
     * @param context 用户自定义上下文指针
     */
    void(*init)(HfsmMachine* m, const HfsmState* initial_state, void* context);
    /**
     * @brief 发布事件到状态机
     * @param m 状态机实例指针
     * @param event_id 事件标识符
     * @param data 事件数据指针
     * @return 发布成功返回 true，失败返回 false
     */
    bool(*post)(HfsmMachine* m, HfsmEventId event_id, void* data);
    /**
     * @brief 清除状态机的待处理事件
     * @param m 状态机实例指针
     */
    void(*clear)(HfsmMachine* m);
    /**
     * @brief 处理状态机的待处理事件
     * @param m 状态机实例指针
     */
    void(*process)(HfsmMachine* m);
    /**
     * @brief 获取状态机的当前状态
     * @param m 状态机实例指针
     * @return 当前状态指针，若 m 为 NULL 则返回 NULL
     */
    const HfsmState* (*current)(const HfsmMachine* m);
    /**
     * @brief 获取状态机的上下文指针
     * @param m 状态机实例指针
     * @return 上下文指针，若 m 为 NULL 则返回 NULL
     */
    void* (*context)(HfsmMachine* m);
    /**
     * @brief 获取状态机的上下文指针（只读）
     * @param m 状态机实例指针
     * @return 上下文指针，若 m 为 NULL 则返回 NULL
     */
    const void* (*const_context)(const HfsmMachine* m);
    /**
     * @brief 判断一个状态是否是另一个状态的子状态
     * @param state 待判断状态指针
     * @param ancestor 祖先状态指针
     * @return 若 state 是 ancestor 的子状态返回 true，否则返回 false
     */
    bool(*is_descendant_of)(const HfsmState* state, const HfsmState* ancestor);
    /**
     * @brief 从当前状态转换到目标状态
     * @param m 状态机实例指针
     * @param target_state 目标状态指针
     * @return 转换成功返回 true，失败返回 false
     */
    bool(*transition)(HfsmMachine* m, const HfsmState* target_state);
    /**
     * @brief 判断状态机是否有待处理事件
     * @param m 状态机实例指针
     * @return 若有待处理事件返回 true，否则返回 false
     */
    bool(*has_pending)(const HfsmMachine* m);
    /**
     * @brief 状态机处理结果接口函数集合
     * @param ignore 忽略事件
     * @param handled 事件已处理但不进行状态转换
     * @param transition 进行状态转换，参数为目标状态指针
     */
    struct {
        /**
         * @brief 忽略事件
         * @return HfsmResult 处理结果结构体，type 字段为 HFSM_RES_IGNORE
         */
        HfsmResult(*ignore)(void);
        /**
         * @brief 事件已处理但不进行状态转换
         * @return HfsmResult 处理结果结构体，type 字段为 HFSM_RES_HANDLED
         */
        HfsmResult(*handled)(void);
        /**
         * @brief 进行状态转换
         * @param next_state 目标状态指针
         * @return HfsmResult 处理结果结构体，type 字段为 HFSM_RES_TRANSITION，next_state 字段指向目标状态
         */
        HfsmResult(*transition)(const HfsmState* next_state);
    } res;
} hfsm_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void hfsm_init(HfsmMachine* m, const HfsmState* initial_state, void* context);
bool hfsm_post(HfsmMachine* m, HfsmEventId event_id, void* data);
void hfsm_clear(HfsmMachine* m);
void hfsm_process(HfsmMachine* m);
const HfsmState* hfsm_get_current_state(const HfsmMachine* m);
void* hfsm_get_context(HfsmMachine* m);
const void* hfsm_get_const_context(const HfsmMachine* m);
bool hfsm_is_descendant_of(const HfsmState* state, const HfsmState* ancestor);
bool hfsm_transition(HfsmMachine* m, const HfsmState* target_state);
bool hfsm_has_pending_event(const HfsmMachine* m);
HfsmResult hfsm_res_ignore(void);
HfsmResult hfsm_res_handled(void);
HfsmResult hfsm_res_transition(const HfsmState* next_state);

#endif
