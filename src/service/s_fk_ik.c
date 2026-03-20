#include "s_fk_ik.h"

#include <math.h>
#include <string.h>

#include "tools/matrix.h"

// ! ========================= 变 量 声 明 ========================= ! //

/// @brief 机械臂的 MDH 参数
static ArmMDH_t arm_mdh = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void get_tf_matrix(Matrix* T, float alpha, float a, float theta, float d);
static void fk_compute(const Matrix* const q, Matrix* const out);
static void matrix_to_pose(const Matrix* const T, Pose_t* pose);

static void joints_to_array(const SixDofJoint_t* j, Matrix* q);
static void array_to_joints(const Matrix* q, SixDofJoint_t* j);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化机械臂的 MDH 参数
 * @param mdh 机械臂的 MDH 参数
 * @return ArmErrorCode_t 错误码
 */
ArmErrorCode_t s_six_dof_init(const ArmMDH_t* mdh) {
    if(mdh == NULL) return ARM_ERROR;
    arm_mdh = *mdh;
    return ARM_SUCCESS;
}

/**
 * @brief 计算机械臂的正运动学
 * @param joints 机械臂的关节角度
 * @param pose 输出的末端位姿
 * @return ArmErrorCode_t 错误码
 */
ArmErrorCode_t s_six_dof_fk(const SixDofJoint_t* joints, Pose_t* pose) {
    if(joints == NULL || pose == NULL) return ARM_ERROR;

    matrix_create(q, 6, 1);
    matrix_create(T, 4, 4);

    joints_to_array(joints, &q);
    fk_compute(&q, &T);
    matrix_to_pose(&T, pose);

    return ARM_SUCCESS;
}

/**
 * @brief 计算机械臂的逆运动学，使用数值雅可比法
 * @param pose 目标末端位姿
 * @param joints 输出的关节角度解
 * @param current_joints 当前关节角度，作为初始猜测
 * @return ArmErrorCode_t 错误码
 */
ArmErrorCode_t s_six_dof_ik(const Pose_t* pose, SixDofJoint_t* joints, const SixDofJoint_t* current_joints) {
    if(!pose || !joints || !current_joints)
        return ARM_ERROR;

    matrix_create(q, 6, 1);
    joints_to_array(current_joints, &q);

    matrix_create(T, 4, 4);
    matrix_create(err, 6, 1);
    matrix_create(J, 6, 6);
    matrix_create(JT, 6, 6);
    matrix_create(JJT, 6, 6);
    matrix_create(inv, 6, 6);
    matrix_create(tmp, 6, 6);
    matrix_create(dq, 6, 1);

    float eps = 1e-5f;
    float lambda = 1e-4f;
    float x, y, z;
    float x2, y2, z2;
    float alpha = 0.5f;

    matrix_create(q_tmp, 6, 1);
    matrix_create(T2, 4, 4);
    matrix_identity_create(L, 6);
    matrix_scalar_mul(&L, lambda, &L);

    for(unsigned int iter = 0; iter < 200; iter++) {

        fk_compute(&q, &T);

        matrix_get(&T, 0, 3, &x);
        matrix_get(&T, 1, 3, &y);
        matrix_get(&T, 2, 3, &z);

        matrix_set(&err, 0, 0, pose->position.x - x);
        matrix_set(&err, 1, 0, pose->position.y - y);
        matrix_set(&err, 2, 0, pose->position.z - z);
        matrix_set(&err, 3, 0, 0.0f);
        matrix_set(&err, 4, 0, 0.0f);
        matrix_set(&err, 5, 0, 0.0f);

        float norm = sqrtf(err.pdata[0] * err.pdata[0] +
            err.pdata[1] * err.pdata[1] +
            err.pdata[2] * err.pdata[2]);

        if(norm < 1e-4f) {
            array_to_joints(&q, joints);
            return ARM_SUCCESS;
        }

        for(unsigned int i = 0; i < 6; i++) {
            matrix_copy(&q, &q_tmp);
            q_tmp.pdata[i] += eps;

            fk_compute(&q_tmp, &T2);

            matrix_get(&T2, 0, 3, &x2);
            matrix_get(&T2, 1, 3, &y2);
            matrix_get(&T2, 2, 3, &z2);

            matrix_set(&J, 0, i, (x2 - x) / eps);
            matrix_set(&J, 1, i, (y2 - y) / eps);
            matrix_set(&J, 2, i, (z2 - z) / eps);

            matrix_set(&J, 3, i, 0.0f);
            matrix_set(&J, 4, i, 0.0f);
            matrix_set(&J, 5, i, 0.0f);
        }

        matrix_transpose(&J, &JT);
        matrix_mul(&JT, &J, &JJT);
        matrix_add(&JJT, &L, &JJT);

        if(matrix_inverse(&JJT, &inv) != MATRIX_SUCCESS) return ARM_ERROR_SINGULARITY;
        matrix_mul(&inv, &JT, &tmp);
        matrix_mul(&tmp, &err, &dq);

        for(unsigned int i = 0; i < 6; i++) {
            q.pdata[i] += alpha * dq.pdata[i];

            if(q.pdata[i] < arm_mdh.qmin[i]) q.pdata[i] = arm_mdh.qmin[i];
            if(q.pdata[i] > arm_mdh.qmax[i]) q.pdata[i] = arm_mdh.qmax[i];
        }
    }

    return ARM_ERROR_OUT_OF_REACH;
}

/**
 * @brief 计算机械臂的所有逆运动学解，当前实现仅返回一个解
 * @param pose 目标末端位姿
 * @param joints 输出的所有关节角度解
 * @return ArmErrorCode_t 错误码
 */
ArmErrorCode_t s_six_dof_all_ik(const Pose_t* pose, SixDofJointAll_t* joints) {
    if(!pose || !joints) return ARM_ERROR;

    joints->num_solutions = 0;

    SixDofJoint_t seed = { 0 };
    if(s_six_dof_ik(pose, &joints->solution_1, &seed) == ARM_SUCCESS) {
        joints->num_solutions = 1;
        return ARM_SUCCESS;
    }

    return ARM_ERROR;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 将关节结构体转换为数组形式，便于计算
 * @param j 关节结构体
 * @param q 输出的关节角度数组
 */
static void joints_to_array(const SixDofJoint_t* j, Matrix* q) {
    matrix_set(q, 0, 0, j->joint_1);
    matrix_set(q, 1, 0, j->joint_2);
    matrix_set(q, 2, 0, j->joint_3);
    matrix_set(q, 3, 0, j->joint_4);
    matrix_set(q, 4, 0, j->joint_5);
    matrix_set(q, 5, 0, j->joint_6);
}

/**
 * @brief 将关节角度数组转换回结构体形式
 * @param q 关节角度数组
 * @param j 输出的关节结构体
 */
static void array_to_joints(const Matrix* q, SixDofJoint_t* j) {
    matrix_get(q, 0, 0, &j->joint_1);
    matrix_get(q, 1, 0, &j->joint_2);
    matrix_get(q, 2, 0, &j->joint_3);
    matrix_get(q, 3, 0, &j->joint_4);
    matrix_get(q, 4, 0, &j->joint_5);
    matrix_get(q, 5, 0, &j->joint_6);
}


/**
 * @brief 根据 MDH 参数生成变换矩阵
 * @param T 输出的 4x4 变换矩阵
 * @param alpha 关节 i-1 的扭转角
 * @param a 关节 i-1 的连杆长度
 * @param theta 关节 i 的关节角
 * @param d 关节 i 的连杆偏移
 */
static void get_tf_matrix(Matrix* T, float alpha, float a, float theta, float d) {
    matrix_set(T, 0, 0, cosf(theta));
    matrix_set(T, 0, 1, -sinf(theta));
    matrix_set(T, 0, 2, 0);
    matrix_set(T, 0, 3, a);

    matrix_set(T, 1, 0, sinf(theta) * cosf(alpha));
    matrix_set(T, 1, 1, cosf(theta) * cosf(alpha));
    matrix_set(T, 1, 2, -sinf(alpha));
    matrix_set(T, 1, 3, -d * sinf(alpha));

    matrix_set(T, 2, 0, sinf(theta) * sinf(alpha));
    matrix_set(T, 2, 1, cosf(theta) * sinf(alpha));
    matrix_set(T, 2, 2, cosf(alpha));
    matrix_set(T, 2, 3, d * cosf(alpha));

    matrix_set(T, 3, 0, 0);
    matrix_set(T, 3, 1, 0);
    matrix_set(T, 3, 2, 0);
    matrix_set(T, 3, 3, 1);
}

/**
 * @brief 计算机械臂的正运动学，得到末端的变换矩阵
 * @param q 关节角度数组
 * @param T_out 输出的末端变换矩阵
 */
static void fk_compute(const Matrix* const q, Matrix* const out) {
    matrix_identity_create(T, 4);
    matrix_identity_create(Ti, 4);
    matrix_identity_create(temp, 4);

    for(int i = 0; i < 6; i++) {
        get_tf_matrix(&Ti,
            arm_mdh.alpha[i],
            arm_mdh.a[i],
            q->pdata[i] + arm_mdh.offset[i],
            arm_mdh.d[i]);

        matrix_mul(&T, &Ti, &temp);
        matrix_copy(&temp, &T);
    }
    matrix_copy(&T, out);
}

/**
 * @brief 将 4x4 变换矩阵转换为位姿结构体
 * @param T 输入的 4x4 变换矩阵
 * @param pose 输出的位姿结构体
 */
static void matrix_to_pose(const Matrix* const T, Pose_t* pose) {
    matrix_get(T, 0, 3, &pose->position.x);
    matrix_get(T, 1, 3, &pose->position.y);
    matrix_get(T, 2, 3, &pose->position.z);

    float r11 = 0, r12 = 0, r13 = 0;
    float r21 = 0, r22 = 0, r23 = 0;
    float r31 = 0, r32 = 0, r33 = 0;
    matrix_get(T, 0, 0, &r11);
    matrix_get(T, 0, 1, &r12);
    matrix_get(T, 0, 2, &r13);

    matrix_get(T, 1, 0, &r21);
    matrix_get(T, 1, 1, &r22);
    matrix_get(T, 1, 2, &r23);

    matrix_get(T, 2, 0, &r31);
    matrix_get(T, 2, 1, &r32);
    matrix_get(T, 2, 2, &r33);

    float qw = sqrtf(1.0f + r11 + r22 + r33) / 2.0f;

    pose->orientation.w = qw;
    pose->orientation.x = (r23 - r32) / (4 * qw);
    pose->orientation.y = (r31 - r13) / (4 * qw);
    pose->orientation.z = (r12 - r21) / (4 * qw);
}
