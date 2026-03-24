#ifndef _fsm_h_
#define _fsm_h_



// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief FSM 状态表(name, parent, entry, exit, action)，由 X-Macro 定义，方便维护和扩展
 * @param NORMAL 正常状态
 * @param IDLE 空闲状态
 * @param SEARCHING 搜索状态
 * @param IK_POSE 逆运动学计算状态
 * @param MOVING 运动状态
 * @param LASERING 激光执行状态
 * @param ERROR 错误状态
 */
#define STATE_TABLE \
    SX(normal,      0,              0,                  0,              normal_action)      \
    SX(idle,        &state_normal,  0,                  0,              idle_action)        \
    SX(searching,   &state_normal,  searching_entry,    0,              searching_action)   \
    SX(ik_pose,     &state_normal,  0,                  0,              ik_pose_action)     \
    SX(moving,      &state_normal,  0,                  0,              moving_action)      \
    SX(lasering,    &state_normal,  laser_entry,        laser_exit,     lasering_action)    \
    SX(error,       0,              error_entry,        0,              0)

/**
 * @brief FSM 事件表(name, value)，由 X-Macro 定义，方便维护和扩展
 * @param START_SEARCH 开始搜索事件
 * @param SEARCH_COMPLETE 搜索完成事件
 * @param IK_COMPLETE 逆运动学计算完成事件
 * @param MOVE_COMPLETE 运动完成事件
 * @param LASER_COMPLETE 激光执行完成事件
 * @param DETECT_ERROR 发现错误事件
 */
#define EVENT_TABLE \
    EX(NONE, 0) \
    EX(START_SEARCH, 1) \
    EX(SEARCH_COMPLETE, 2) \
    EX(IK_COMPLETE, 3) \
    EX(MOVE_COMPLETE, 4) \
    EX(LASER_COMPLETE, 5) \
    EX(DETECT_ERROR, 6) \

/**
 * @brief FSM 事件枚举类型，由 X-Macro 自动生成，表示状态机中的各种事件
 */
#define EX(name, value) EVENT_##name = value, 
typedef enum {
    EVENT_TABLE
} Event;
#undef EX

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
    State* (*handle)(Event event);
    /**
     * @brief 状态进入动作函数
     */
    void (*entry)(void);
    /**
     * @brief 状态退出动作函数
     */
    void (*exit)(void);
    /**
     * @brief 状态持续动作函数
     */
    void (*action)(void);

/// private:
    /**
     * @brief 父状态指针，用于实现层次状态机，指向当前状态的父状态，如果没有父状态则为 NULL
     */
    const State* _parent_;
};

/// @brief 当前事件，由 FSM 内部维护
extern Event current_event;

/// @brief 当前状态指针，由 FSM 内部维护
extern State* current_state;

/**
 * @brief FSM 状态声明，由 X-Macro 自动生成，表示状态机中的各种状态
 */
#define SX(name, parent, entry, exit, action) extern State state_##name;
STATE_TABLE
#undef SX

// ! ========================= 接 口 函 数 声 明 ========================= ! //



#endif
