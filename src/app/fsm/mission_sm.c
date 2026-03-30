#include "mission_sm.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app/packet_parser.h"
#include "domain/arm_kine.h"

typedef union {
    void* ptr;
    const WeedData* weed;
} MissionEventData;
#define HFSM_EVENT_DATA_TYPE MissionEventData
#include "infra/hfsm/hfsm_core.h"
#include "infra/delay.h"
#include "infra/matrix.h"

#include "platform/led.h"

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief mission 状态机实例
 * @param init 初始化函数，参数为 WiFi 连接信息指针
 * @param post 事件发布函数，参数为事件 ID 和可选数据指针
 * @param process 状态机处理函数，需在主循环中定期调用
 * @param update_dm_feedback 电机反馈更新函数，参数为电机反馈数据
 * @param state 获取当前状态名称函数，返回字符串
 * @param context 获取当前业务上下文函数，返回指向 MissionContext 的指针
 */
const struct MissionInstance mission_instance = {
    .init = mission_sm_init,
    .post = mission_sm_post,
    .process = mission_sm_process,
    .update_dm_feedback = mission_sm_update_dm_feedback,
    .state = mission_sm_state,
    .context = mission_sm_context,
    {
        #define EX(name) .name = MISSION_EVENT_##name,
        #include "mission_def_event.inc"
        #undef EX
    }
};

static HfsmMachine s_machine;
static MissionContext s_ctx;

// ! ========================= 状 态 机 声 明 ========================= ! //

/**
 * @brief 业务状态定义
 */
#define SX(name, parent) \
    static HfsmResult handle_##name(HfsmMachine* m, const HfsmEvent* e); \
    static void entry_##name(HfsmMachine* m); \
    static void exit_##name(HfsmMachine* m); \
    static void action_##name(HfsmMachine* m); \
    static const HfsmState state_##name;
#include "mission_def_state.inc"
#undef SX

/**
 * @brief 定义状态结构体
 */
#define SX(n, p) \
    static const HfsmState state_##n = { \
        .name = #n, \
        .parent = p, \
        .handle = handle_##n, \
        .entry = entry_##n, \
        .exit = exit_##n, \
        .action = action_##n \
    };
#include "mission_def_state.inc"
#undef SX

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static MissionContext* ctx_of(HfsmMachine* m);
static const MissionContext* const_ctx_of(const HfsmMachine* m);

static void reset_context(MissionContext* ctx);
static void set_error(MissionContext* ctx, const char* error);

static bool post_event(MissionEvent event, void* data);

static void clear_dm_feedback_valid(MissionContext* ctx);
static bool all_feedback_ready(const MissionContext* ctx);
static bool joints_reached_target(const MissionContext* ctx, float pos_err);
static bool joints_reached_zero(const MissionContext* ctx, float pos_err);

static bool generate_aim_pose(Vector3 source, Vector3 target, Pose* out);
static bool search_reachable_pose(MissionContext* ctx);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化状态机，设置初始状态和上下文
 * @param info WiFi 连接信息指针，将存储在上下文中供后续使用
 */
void mission_sm_init(WifiBtConnectInfo* info) {
    s_ctx = (MissionContext){ 0 };
    s_ctx.wifi_info = info;
    led.on();

    hfsm_core.init(&s_machine, &state_init, &s_ctx);
}

/**
 * @brief 发布事件到状态机
 * @param event 事件 ID，需为 MissionEvent 枚举值
 * @param data 可选数据指针，事件处理函数可根据需要使用
 * @return true 如果事件成功发布并被状态机接受，false 如果事件无效或被拒绝
 */
bool mission_sm_post(MissionEvent event, void* data) {
    return hfsm_core.post(&s_machine, (HfsmEventId)event, data);
}

/**
 * @brief 状态机处理函数，需在主循环中定期调用以处理事件和状态转换
 */
void mission_sm_process(void) {
    hfsm_core.process(&s_machine);
}

/**
 * @brief 更新电机反馈数据，状态机内部使用这些数据进行状态转换和动作控制
 * @param feedback 电机反馈数据结构，包含电机 ID、位置、速度等信息
 */
void mission_sm_update_dm_feedback(DmMotorFeedback feedback) {
    if(feedback.id >= 1 && feedback.id <= 6) {
        s_ctx.dm_fb[feedback.id] = feedback;
        s_ctx.dm_fb_valid[feedback.id] = true;
    }
}

/**
 * @brief 获取当前状态名称，主要用于调试和日志记录
 * @return 当前状态的字符串名称，如果状态机未初始化或状态无效则返回 "null"
 */
const char* mission_sm_state(void) {
    const HfsmState* s = hfsm_core.state(&s_machine);
    return s ? s->name : "null";
}

/**
 * @brief 获取当前业务上下文，包含错误信息、WiFi 连接信息、目标杂草数据、目标位姿、IK 求解结果、电机反馈等
 * @return 指向当前业务上下文的指针，调用者应避免修改返回的上下文数据
 */
const MissionContext* mission_sm_context(void) {
    return const_ctx_of(&s_machine);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 获取可修改的业务上下文指针
 * @param m 状态机指针
 * @return 指向业务上下文的指针，调用者可修改返回的上下文数据
 */
static MissionContext* ctx_of(HfsmMachine* m) {
    return (MissionContext*)hfsm_core.context(m);
}

/**
 * @brief 获取只读的业务上下文指针
 * @param m 状态机指针
 * @return 指向业务上下文的指针，调用者应避免修改返回的上下文数据
 */
static const MissionContext* const_ctx_of(const HfsmMachine* m) {
    return (const MissionContext*)hfsm_core.const_context(m);
}

/**
 * @brief 重置业务上下文，清除除 WiFi 连接信息外的所有数据
 * @param ctx 业务上下文指针
 */
static void reset_context(MissionContext* ctx) {
    WifiBtConnectInfo* wifi_info = ctx->wifi_info;
    *ctx = (MissionContext){ 0 };
    ctx->wifi_info = wifi_info;
}

/**
 * @brief 设置当前错误信息，状态机进入 fault 状态时会记录错误信息以供调试
 * @param ctx 业务上下文指针
 * @param error 错误信息字符串，调用者应确保字符串在上下文使用期间有效
 */
static void set_error(MissionContext* ctx, const char* error) {
    if(ctx == NULL) return;
    ctx->error = error;
}

/**
 * @brief 发布事件到状态机的简化函数，封装了事件 ID 和数据参数
 * @param event 事件 ID，需为 MissionEvent 枚举值
 * @param data 可选数据指针，事件处理函数可根据需要使用
 * @return true 如果事件成功发布并被状态机接受，false 如果事件无效或被拒绝
 */
static bool post_event(MissionEvent event, void* data) {
    return mission_sm_post(event, data);
}

/**
 * @brief 发布事件到状态机的简化函数，适用于无数据事件
 * @param event 事件 ID，需为 MissionEvent 枚举值
 * @return true 如果事件成功发布并被状态机接受，false 如果事件无效或被拒绝
 */
static void clear_dm_feedback_valid(MissionContext* ctx) {
    for(uint8_t i = 1; i <= 6; ++i) {
        ctx->dm_fb_valid[i] = false;
    }
}

/**
 * @brief 检查所有电机反馈数据是否准备就绪
 * @param ctx 业务上下文指针
 * @return true 如果所有电机反馈数据均有效，false 如果至少有一个电机反馈数据无效
 */
static bool all_feedback_ready(const MissionContext* ctx) {
    for(uint8_t i = 1; i <= 6; ++i) {
        if(!ctx->dm_fb_valid[i]) return false;
    }
    return true;
}

/**
 * @brief 检查当前电机位置是否已达到目标位姿，允许一定的误差范围
 * @param ctx 业务上下文指针，包含当前电机反馈和 IK 求解结果
 * @param pos_err 位置误差容限，单位为弧度，通常设置为 0.1 弧度左右
 * @return true 如果所有电机位置均在目标位姿的误差范围内，false 如果至少有一个电机位置未达到目标位姿
 */
static bool joints_reached_target(const MissionContext* ctx, float pos_err) {
    return
        fabsf(ctx->dm_fb[1].pos - ctx->ik_result.joint_1) <= pos_err &&
        fabsf(ctx->dm_fb[2].pos - ctx->ik_result.joint_2) <= pos_err &&
        fabsf(ctx->dm_fb[3].pos - ctx->ik_result.joint_3) <= pos_err &&
        fabsf(ctx->dm_fb[4].pos - ctx->ik_result.joint_4) <= pos_err &&
        fabsf(ctx->dm_fb[5].pos - ctx->ik_result.joint_5) <= pos_err &&
        fabsf(ctx->dm_fb[6].pos - ctx->ik_result.joint_6) <= pos_err;
}

/**
 * @brief 检查当前电机位置是否已达到零位，允许一定的误差范围
 * @param ctx 业务上下文指针，包含当前电机反馈
 * @param pos_err 位置误差容限，单位为弧度，通常设置为 0.1 弧度左右
 * @return true 如果所有电机位置均在零位的误差范围内，false 如果至少有一个电机位置未达到零位
 */
static bool joints_reached_zero(const MissionContext* ctx, float pos_err) {
    return
        fabsf(ctx->dm_fb[1].pos - 0.0f) <= pos_err &&
        fabsf(ctx->dm_fb[2].pos - 0.0f) <= pos_err &&
        fabsf(ctx->dm_fb[3].pos - 0.0f) <= pos_err &&
        fabsf(ctx->dm_fb[4].pos - 0.0f) <= pos_err &&
        fabsf(ctx->dm_fb[5].pos - 0.0f) <= pos_err &&
        fabsf(ctx->dm_fb[6].pos - 0.0f) <= pos_err;
}

/**
 * @brief 生成瞄准位姿，基于目标杂草位置和理想手臂位置计算一个可达的位姿
 * @param source 理想手臂位置，通常在目标上方
 * @param target 目标杂草位置
 * @param out 输出位姿指针，函数成功时填充为计算得到的瞄准位姿
 * @return true 如果成功生成瞄准位姿，false 如果计算失败（如目标过近或过远导致无法生成有效位姿）
 */
static bool generate_aim_pose(Vector3 source, Vector3 target, Pose* out) {
    if(out == NULL) return false;

    out->position.x = source.x;
    out->position.y = source.y;
    out->position.z = source.z;

    Vector3 aim_dir = {
        .x = target.x - source.x,
        .y = target.y - source.y,
        .z = target.z - source.z
    };

    float aim_norm = 0.0f;
    if(vec3_norm(&aim_dir, &aim_norm) != MATRIX_SUCCESS) return false;
    if(aim_norm < 1e-4f) return false;
    if(vec3_normalize(&aim_dir, &aim_dir) != MATRIX_SUCCESS) return false;

    Vector3 z_axis = aim_dir;
    Vector3 world_up = { 0.0f, 0.0f, 1.0f };
    Vector3 x_axis = { 0 };
    float x_norm = 0.0f;

    if(vec3_cross(&world_up, &z_axis, &x_axis) != MATRIX_SUCCESS) return false;
    if(vec3_norm(&x_axis, &x_norm) != MATRIX_SUCCESS) return false;

    if(x_norm < 1e-3f) {
        world_up = (Vector3){ 0.0f, 1.0f, 0.0f };
        if(vec3_cross(&world_up, &z_axis, &x_axis) != MATRIX_SUCCESS) return false;
        if(vec3_norm(&x_axis, &x_norm) != MATRIX_SUCCESS) return false;
        if(x_norm < 1e-3f) return false;
    }

    if(vec3_normalize(&x_axis, &x_axis) != MATRIX_SUCCESS) return false;

    Vector3 y_axis = { 0 };
    if(vec3_cross(&z_axis, &x_axis, &y_axis) != MATRIX_SUCCESS) return false;
    if(vec3_normalize(&y_axis, &y_axis) != MATRIX_SUCCESS) return false;

    float R_data[3 * 3];
    Matrix R;
    if(matrix(&R, 3, 3, R_data) != MATRIX_SUCCESS) return false;

    R.pdata[0 * 3 + 0] = x_axis.x;
    R.pdata[0 * 3 + 1] = y_axis.x;
    R.pdata[0 * 3 + 2] = z_axis.x;

    R.pdata[1 * 3 + 0] = x_axis.y;
    R.pdata[1 * 3 + 1] = y_axis.y;
    R.pdata[1 * 3 + 2] = z_axis.y;

    R.pdata[2 * 3 + 0] = x_axis.z;
    R.pdata[2 * 3 + 1] = y_axis.z;
    R.pdata[2 * 3 + 2] = z_axis.z;

    Quaternion quat;
    if(matrix_to_quat(&R, (float*)&quat) != MATRIX_SUCCESS) return false;
    if(quat_normalize((const float*)&quat, (float*)&quat) != MATRIX_SUCCESS) return false;

    out->orientation = quat;
    return true;
}

/**
 * @brief 搜索可达的瞄准位姿，基于理想位姿在周围进行微调搜索，直到找到一个 IK 可解的位姿
 * @param ctx 业务上下文指针，包含目标杂草数据和用于存储搜索结果的字段
 * @return true 如果找到可达的瞄准位姿并成功计算 IK 解，false 如果搜索失败
 */
static bool search_reachable_pose(MissionContext* ctx) {
    Vector3 ideal_pos = { 0.0f, 0.2f, 0.3f };
    Vector3 target_pos = { ctx->weed.x, ctx->weed.y, ctx->weed.z };
    SixDofJoint seed_joints = { 0.0f, 0.6f, 1.14f, -1.06f, 0.0f, 0.0f };

    float offsets[] = { 0.0f, 0.02f, -0.02f, 0.05f, -0.05f };
    const int num_offsets = (int)(sizeof(offsets) / sizeof(offsets[0]));

    printf("[FSM] 开始瞄准位姿搜索，目标:(%.2f, %.2f, %.2f)\r\n",
        target_pos.x, target_pos.y, target_pos.z);

    for(int i = 0; i < num_offsets; ++i) {
        for(int j = 0; j < num_offsets; ++j) {
            for(int k = 0; k < num_offsets; ++k) {
                Vector3 search_pos = ideal_pos;
                search_pos.x += offsets[i];
                search_pos.y += offsets[j];
                search_pos.z += offsets[k];

                Pose candidate_pose;
                if(!generate_aim_pose(search_pos, target_pos, &candidate_pose)) {
                    continue;
                }

                SixDofJoint probe_joints = { 0 };
                if(arm.ik(&candidate_pose, &probe_joints, &seed_joints, IK_MODE_FULL_POSE) == ARM_STATUS_SUCCESS) {
                    ctx->target_pose = candidate_pose;
                    ctx->ik_result = probe_joints;
                    return true;
                }
            }
        }
    }

    return false;
}

// ! ========================= 状 态 实 现 ========================= ! //


/**
 * @brief init 状态
 */
static HfsmResult handle_init(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    switch((MissionEvent)e->id) {
        case MISSION_EVENT_INIT_COMPLETE:
            return hfsm_core.res.transition(&state_idle);
        default:
            return hfsm_core.res.ignore();
    }
}
static void entry_init(HfsmMachine* m) {
    reset_context(ctx_of(m));
}
static void exit_init(HfsmMachine* m) {
    (void)m;
}
static void action_init(HfsmMachine* m) {
    (void)m;
    post_event(MISSION_EVENT_INIT_COMPLETE, NULL);
}


/**
 * @brief active 状态
 */
static HfsmResult handle_active(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    switch((MissionEvent)e->id) {
        case MISSION_EVENT_DETECT_ERROR:
            return hfsm_core.res.transition(&state_fault);
        default:
            return hfsm_core.res.ignore();
    }
}
static void entry_active(HfsmMachine* m) {
    (void)m;
}
static void exit_active(HfsmMachine* m) {
    (void)m;
}
static void action_active(HfsmMachine* m) {
    (void)m;
}


/**
 * @brief idle 状态
 */
static HfsmResult handle_idle(HfsmMachine* m, const HfsmEvent* e) {
    MissionContext* ctx = ctx_of(m);

    switch((MissionEvent)e->id) {
        case MISSION_EVENT_START_SEARCH:
        {
            if(e->data.weed == NULL) {
                set_error(ctx, "未提供杂草数据");
                return hfsm_core.res.transition(&state_fault);
            }
            ctx->weed = *e->data.weed;
            return hfsm_core.res.transition(&state_plan_aim);
        }
        case MISSION_EVENT_FINISH:
            return hfsm_core.res.transition(&state_finish);
        default:
            return hfsm_core.res.ignore();
    }
}
static void entry_idle(HfsmMachine* m) {
    (void)m;
    printf("[FSM] 进入空闲状态，等待指令...\r\n");
}
static void exit_idle(HfsmMachine* m) {
    (void)m;
}
static void action_idle(HfsmMachine* m) {
    (void)m;
}


/**
 * @brief plan_aim 状态
 */
static HfsmResult handle_plan_aim(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    switch((MissionEvent)e->id) {
        case MISSION_EVENT_SEARCH_COMPLETE:
            return hfsm_core.res.transition(&state_moving);
        default:
            return hfsm_core.res.ignore();
    }
}
static void entry_plan_aim(HfsmMachine* m) {
    MissionContext* ctx = ctx_of(m);

    printf("[FSM] 捕获目标 ID：%d，坐标：x=%.2f, y=%.2f, z=%.2f, confidence=%.2f\r\n",
        ctx->weed.id, ctx->weed.x, ctx->weed.y, ctx->weed.z, ctx->weed.confidence);
}
static void exit_plan_aim(HfsmMachine* m) {
    (void)m;
}
static void action_plan_aim(HfsmMachine* m) {
    MissionContext* ctx = ctx_of(m);

    if(!search_reachable_pose(ctx)) {
        set_error(ctx, "IK 网格搜索无解");
        post_event(MISSION_EVENT_DETECT_ERROR, NULL);
        return;
    }

    printf("[FSM] 已找到可达瞄准位姿：position=(%.2f, %.2f, %.2f), orientation=(%.2f, %.2f, %.2f)\r\n",
        ctx->target_pose.position.x,
        ctx->target_pose.position.y,
        ctx->target_pose.position.z,
        ctx->target_pose.orientation.x,
        ctx->target_pose.orientation.y,
        ctx->target_pose.orientation.z);
    printf("[FSM] 可达瞄准位姿六轴关节角：joint_1=%.2f, joint_2=%.2f, joint_3=%.2f, joint_4=%.2f, joint_5=%.2f, joint_6=%.2f\r\n",
        ctx->ik_result.joint_1,
        ctx->ik_result.joint_2,
        ctx->ik_result.joint_3,
        ctx->ik_result.joint_4,
        ctx->ik_result.joint_5,
        ctx->ik_result.joint_6);

    post_event(MISSION_EVENT_SEARCH_COMPLETE, NULL);
}


/**
 * @brief moving 状态
 */
static HfsmResult handle_moving(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    switch((MissionEvent)e->id) {
        case MISSION_EVENT_MOVE_COMPLETE:
            return hfsm_core.res.transition(&state_lasering);
        default:
            return hfsm_core.res.ignore();
    }
}
static void entry_moving(HfsmMachine* m) {
    MissionContext* ctx = ctx_of(m);

    printf("[FSM] 进入移动状态...\r\n");

    dm.set_pos_spd(0x01, ctx->ik_result.joint_1, 1.57f);
    dm.set_pos_spd(0x02, ctx->ik_result.joint_2, 1.57f);
    dm.set_pos_spd(0x03, ctx->ik_result.joint_3, 1.57f);
    dm.set_pos_spd(0x04, ctx->ik_result.joint_4, 1.57f);
    dm.set_pos_spd(0x05, ctx->ik_result.joint_5, 1.57f);
    dm.set_pos_spd(0x06, ctx->ik_result.joint_6, 1.57f);

    clear_dm_feedback_valid(ctx);
    ctx->move_timer = 0;
}
static void exit_moving(HfsmMachine* m) {
    (void)m;
}
static void action_moving(HfsmMachine* m) {
    MissionContext* ctx = ctx_of(m);

    if(all_feedback_ready(ctx) && joints_reached_target(ctx, 0.05f)) {
        printf("[FSM] 移动完成，进入激光状态...\r\n");
        post_event(MISSION_EVENT_MOVE_COMPLETE, NULL);
        return;
    }

    if(s_nb_delay_ms(&ctx->move_timer, 5000)) {
        set_error(ctx, "移动超时");
        post_event(MISSION_EVENT_DETECT_ERROR, NULL);
    }
}


/**
 * @brief lasering 状态
 */
static HfsmResult handle_lasering(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    switch((MissionEvent)e->id) {
        case MISSION_EVENT_LASER_COMPLETE:
            return hfsm_core.res.transition(&state_idle);
        default:
            return hfsm_core.res.ignore();
    }
}
static void entry_lasering(HfsmMachine* m) {
    MissionContext* ctx = ctx_of(m);

    ctx->laser_timer = 0;

    if(ctx->wifi_info != NULL) {
        wifi.send(*ctx->wifi_info, "Ready to Laser", (uint16_t)strlen("Ready to Laser"));
    }
}
static void exit_lasering(HfsmMachine* m) {
    (void)m;
    printf("[FSM] 激光完成，返回空闲状态...\r\n");
}
static void action_lasering(HfsmMachine* m) {
    MissionContext* ctx = ctx_of(m);

    if(s_nb_delay_ms(&ctx->laser_timer, 5000)) {
        printf("[FSM] 激光完成超时，自动进入空闲状态...\r\n");
        post_event(MISSION_EVENT_LASER_COMPLETE, NULL);
    }
}


/**
 * @brief finish 状态
 */
static HfsmResult handle_finish(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    switch((MissionEvent)e->id) {
        case MISSION_EVENT_FINISH_COMPLETE:
            return hfsm_core.res.transition(&state_idle);
        default:
            return hfsm_core.res.ignore();
    }
}
static void entry_finish(HfsmMachine* m) {
    MissionContext* ctx = ctx_of(m);

    printf("[FSM] 进入完成状态，执行机械臂复位...\r\n");

    dm.set_pos_spd(0x01, 0.0f, 1.57f);
    dm.set_pos_spd(0x02, 0.0f, 1.57f);
    dm.set_pos_spd(0x03, 0.0f, 1.57f);
    dm.set_pos_spd(0x04, 0.0f, 1.57f);
    dm.set_pos_spd(0x05, 0.0f, 1.57f);
    dm.set_pos_spd(0x06, 0.0f, 1.57f);

    clear_dm_feedback_valid(ctx);
    ctx->finish_timer = 0;
}
static void exit_finish(HfsmMachine* m) {
    (void)m;
}
static void action_finish(HfsmMachine* m) {
    MissionContext* ctx = ctx_of(m);

    if(all_feedback_ready(ctx) && joints_reached_zero(ctx, 0.1f)) {
        printf("[FSM] 复位完成，返回空闲状态...\r\n");
        post_event(MISSION_EVENT_FINISH_COMPLETE, NULL);
        return;
    }

    if(s_nb_delay_ms(&ctx->finish_timer, 10000)) {
        set_error(ctx, "复位超时");
        post_event(MISSION_EVENT_DETECT_ERROR, NULL);
    }
}


/**
 * @brief fault 状态
 */
static HfsmResult handle_fault(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    switch((MissionEvent)e->id) {
        case MISSION_EVENT_ERROR_COMPLETE:
            return hfsm_core.res.transition(&state_init);
        default:
            return hfsm_core.res.ignore();
    }
}
static void entry_fault(HfsmMachine* m) {
    MissionContext* ctx = ctx_of(m);

    printf("[FSM] 错误发生：%s\r\n", ctx->error ? ctx->error : "未知错误");
}
static void exit_fault(HfsmMachine* m) {
    (void)m;
}
static void action_fault(HfsmMachine* m) {
    (void)m;

    static ms_t blink_timer = 0;
    if(s_nb_delay_ms(&blink_timer, 500)) {
        led.toggle();
    }
}
