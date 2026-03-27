#include "app/fsm/mission_sm.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "domain/arm_kine.h"

#include "infra/hfsm.h"
#include "infra/delay.h"
#include "infra/matrix.h"

#include "platform/led.h"

// ! ========================= 变 量 声 明 ========================= ! //

const struct MissionInstance mission_instance = {
    .init = mission_sm_init,
    .post = mission_sm_post,
    .process = mission_sm_process,
    .update_dm_feedback = mission_sm_update_dm_feedback,
    .state = mission_sm_state,
    .context = mission_sm_context
};

static HfsmMachine s_machine;
static MissionContext s_ctx;

// ! ========================= 状 态 机 声 明 ========================= ! //

#define SX(name, parent) \
    static HfsmResult handle_##name(HfsmMachine* m, const HfsmEvent* e); \
    static void entry_##name(HfsmMachine* m); \
    static void exit_##name(HfsmMachine* m); \
    static void action_##name(HfsmMachine* m); \
    static const HfsmState state_##name;
#include "mission_def_state.inc"
#undef SX

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

void mission_sm_init(WifiBtConnectInfo* info) {
    s_ctx = (MissionContext){ 0 };
    s_ctx.wifi_info = info;

    hfsm.init(&s_machine, &state_init, &s_ctx);
}

bool mission_sm_post(MissionEvent event, void* data) {
    return hfsm.post(&s_machine, (HfsmEventId)event, data);
}

void mission_sm_process(void) {
    hfsm.process(&s_machine);
}

void mission_sm_update_dm_feedback(DmMotorFeedback feedback) {
    if(feedback.id >= 1 && feedback.id <= 6) {
        s_ctx.dm_fb[feedback.id] = feedback;
        s_ctx.dm_fb_valid[feedback.id] = true;
    }
}

const char* mission_sm_state(void) {
    const HfsmState* s = hfsm.state(&s_machine);
    return s ? s->name : "null";
}

const MissionContext* mission_sm_context(void) {
    return const_ctx_of(&s_machine);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static MissionContext* ctx_of(HfsmMachine* m) {
    return (MissionContext*)hfsm.context(m);
}

static const MissionContext* const_ctx_of(const HfsmMachine* m) {
    return (const MissionContext*)hfsm.const_context(m);
}

static void reset_context(MissionContext* ctx) {
    WifiBtConnectInfo* wifi_info = ctx->wifi_info;
    *ctx = (MissionContext){ 0 };
    ctx->wifi_info = wifi_info;
}

static void set_error(MissionContext* ctx, const char* error) {
    if(ctx == NULL) return;
    ctx->error = error;
}

static bool post_event(MissionEvent event, void* data) {
    return mission_sm_post(event, data);
}

static void clear_dm_feedback_valid(MissionContext* ctx) {
    for(uint8_t i = 1; i <= 6; ++i) {
        ctx->dm_fb_valid[i] = false;
    }
}

static bool all_feedback_ready(const MissionContext* ctx) {
    for(uint8_t i = 1; i <= 6; ++i) {
        if(!ctx->dm_fb_valid[i]) return false;
    }
    return true;
}

static bool joints_reached_target(const MissionContext* ctx, float pos_err) {
    return
        fabsf(ctx->dm_fb[1].pos - ctx->ik_result.joint_1) <= pos_err &&
        fabsf(ctx->dm_fb[2].pos - ctx->ik_result.joint_2) <= pos_err &&
        fabsf(ctx->dm_fb[3].pos - ctx->ik_result.joint_3) <= pos_err &&
        fabsf(ctx->dm_fb[4].pos - ctx->ik_result.joint_4) <= pos_err &&
        fabsf(ctx->dm_fb[5].pos - ctx->ik_result.joint_5) <= pos_err &&
        fabsf(ctx->dm_fb[6].pos - ctx->ik_result.joint_6) <= pos_err;
}

static bool joints_reached_zero(const MissionContext* ctx, float pos_err) {
    return
        fabsf(ctx->dm_fb[1].pos - 0.0f) <= pos_err &&
        fabsf(ctx->dm_fb[2].pos - 0.0f) <= pos_err &&
        fabsf(ctx->dm_fb[3].pos - 0.0f) <= pos_err &&
        fabsf(ctx->dm_fb[4].pos - 0.0f) <= pos_err &&
        fabsf(ctx->dm_fb[5].pos - 0.0f) <= pos_err &&
        fabsf(ctx->dm_fb[6].pos - 0.0f) <= pos_err;
}

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
 * @brief init
 */
static HfsmResult handle_init(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    switch((MissionEvent)e->id) {
        case MISSION_EVENT_INIT_COMPLETE:
            return hfsm.res.transition(&state_idle);
        default:
            return hfsm.res.ignore();
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
 * @brief active
 */
static HfsmResult handle_active(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    switch((MissionEvent)e->id) {
        case MISSION_EVENT_DETECT_ERROR:
            return hfsm.res.transition(&state_fault);
        default:
            return hfsm.res.ignore();
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
 * @brief idle
 */
static HfsmResult handle_idle(HfsmMachine* m, const HfsmEvent* e) {
    MissionContext* ctx = ctx_of(m);

    switch((MissionEvent)e->id) {
        case MISSION_EVENT_START_SEARCH:
        {
            if(e->data == NULL) {
                set_error(ctx, "未提供杂草数据");
                return hfsm.res.transition(&state_fault);
            }
            ctx->weed = *(const WeedData*)e->data;
            return hfsm.res.transition(&state_plan_aim);
        }
        case MISSION_EVENT_FINISH:
            return hfsm.res.transition(&state_finish);
        default:
            return hfsm.res.ignore();
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
 * @brief plan_aim
 */
static HfsmResult handle_plan_aim(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    switch((MissionEvent)e->id) {
        case MISSION_EVENT_SEARCH_COMPLETE:
            return hfsm.res.transition(&state_moving);
        default:
            return hfsm.res.ignore();
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
 * @brief moving
 */
static HfsmResult handle_moving(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    switch((MissionEvent)e->id) {
        case MISSION_EVENT_MOVE_COMPLETE:
            return hfsm.res.transition(&state_lasering);
        default:
            return hfsm.res.ignore();
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

    if(all_feedback_ready(ctx) && joints_reached_target(ctx, 0.1f)) {
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
 * @brief lasering
 */
static HfsmResult handle_lasering(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    switch((MissionEvent)e->id) {
        case MISSION_EVENT_LASER_COMPLETE:
            return hfsm.res.transition(&state_idle);
        default:
            return hfsm.res.ignore();
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
 * @brief finish
 */
static HfsmResult handle_finish(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    switch((MissionEvent)e->id) {
        case MISSION_EVENT_FINISH_COMPLETE:
            return hfsm.res.transition(&state_idle);
        default:
            return hfsm.res.ignore();
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
 * @brief fault
 */
static HfsmResult handle_fault(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    switch((MissionEvent)e->id) {
        case MISSION_EVENT_ERROR_COMPLETE:
            return hfsm.res.transition(&state_init);
        default:
            return hfsm.res.ignore();
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
