#ifndef _hfsm_config_h_
#define _hfsm_config_h_

/**
 * @brief HFSM 最大状态层级深度
 */
#ifndef HFSM_DEPTH
#define HFSM_DEPTH 8
#endif

/**
 * @brief 状态机进程是否执行所有祖先状态的动作，1 表示执行，0 表示只执行当前状态的动作
 */
#ifndef HFSM_RUN_PARENT_ACTIONS
#define HFSM_RUN_PARENT_ACTIONS 1
#endif

/**
 * @brief 是否启用断言，1 表示启用，0 表示禁用
 */
#ifndef HFSM_ENABLE_ASSERT
#define HFSM_ENABLE_ASSERT 1
#endif

/**
 * @brief TODO: 状态机待处理事件队列的最大长度
 */
// #ifndef HFSM_PENDING_MAX
// #define HFSM_PENDING_MAX 8
// #endif

#endif
