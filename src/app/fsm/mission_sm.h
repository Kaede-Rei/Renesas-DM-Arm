#ifndef _mission_sm_h_
#define _mission_sm_h_

#include <stdbool.h>

#include "app/fsm/mission_context.h"

#include "device/motor.h"
#include "device/wifi_bt.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

#define mission mission_instance

/**
 * @brief 任务状态机事件枚举
 * @param NONE 无事件
 *
 * @param INIT_COMPLETE 初始化完成事件
 *
 * @param START_SEARCH 开始搜索事件
 * @param SEARCH_COMPLETE 搜索完成事件
 * @param MOVE_COMPLETE 运动完成事件
 * @param LASER_COMPLETE 激光完成事件
 * @param FINISH 任务完成事件
 * @param FINISH_COMPLETE 任务完成确认事件
 *
 * @param DETECT_ERROR 检测错误事件
 * @param ERROR_COMPLETE 错误处理完成事件
 */
#define EX(name) MISSION_EVENT_##name,
typedef enum {
#include "mission_def_event.inc"
} MissionEvent;
#undef EX

#define EX(name) const MissionEvent name;
extern const struct MissionInstance {
    /**
     * @brief 初始化函数，参数为 WiFi 连接信息指针
     * @param info WiFi 连接信息指针，包含 SSID 和密码等连接参数
     */
    void(*init)(WifiBtConnectInfo* info);
    /**
     * @brief 事件发布函数，参数为事件 ID 和可选数据指针
     * @param event 事件 ID，需为 MissionEvent 枚举值
     * @param data 事件数据指针，具体数据类型根据事件而定，调用者应确保数据在状态机处理期间有效
     * @return 发布是否成功，成功返回 true，失败返回 false（如事件无效或状态机未初始化）
     */
    bool(*post)(MissionEvent event, void* data);
    /**
     * @brief 状态机处理函数，需在主循环中定期调用以处理事件和状态转换
     */
    void(*process)(void);
    /**
     * @brief 电机反馈更新函数，参数为电机反馈数据
     * @param feedback 电机反馈数据结构，包含电机 ID、位置、速度等信息
     */
    void(*update_dm_feedback)(DmMotorFeedback feedback);
    /**
     * @brief 获取当前状态名称函数，返回字符串
     * @return 当前状态的字符串名称，如果状态机未初始化或状态无效则返回 "null"
     */
    const char* (*state)(void);
    /**
     * @brief 获取当前业务上下文函数，返回指向 MissionContext 的指针
     * @return 指向当前业务上下文的指针，调用者应避免修改返回的上下文数据
     */
    const MissionContext* (*context)(void);
    struct {
#include "mission_def_event.inc"
    };
} mission_instance;
#undef EX

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void mission_sm_init(WifiBtConnectInfo* info);
bool mission_sm_post(MissionEvent event, void* data);
void mission_sm_process(void);
void mission_sm_update_dm_feedback(DmMotorFeedback feedback);

const char* mission_sm_state(void);
const MissionContext* mission_sm_context(void);

#endif
