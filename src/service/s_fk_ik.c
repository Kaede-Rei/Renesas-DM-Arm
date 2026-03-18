#include "s_fk_ik.h"

#include <math.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

/// @brief 机械臂的 MDH 参数
static ArmMDH_t arm_mdh = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void mat4_identity(double* T);
static void mat4_mul(const double* A, const double* B, double* R);
static void get_tf_matrix(double* T, double alpha, double a, double theta, double d);
static void fk_compute(const double* q, double* T_out);
static void matrix_to_pose(const double* T, Pose_t* pose);

static void joints_to_array(const SixDofJoint_t* j, double* q);
static void array_to_joints(const double* q, SixDofJoint_t* j);

static void mat6_mul(const double* A, const double* B, double* R);
static void mat6_vec_mul(const double* A, const double* x, double* y);
static void mat6_identity(double* A);
static int  mat6_inverse(double* A, double* inv);

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

    double q[6];
    double T[16];

    joints_to_array(joints, q);
    fk_compute(q, T);
    matrix_to_pose(T, pose);

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
    if(!pose || !joints || !current_joints) return ARM_ERROR;

    double q[6];
    joints_to_array(current_joints, q);

    for(int iter = 0; iter < 200; iter++) {

        double T[16];
        fk_compute(q, T);

        double err[6] = { 0 };

        err[0] = pose->position.x - T[3];
        err[1] = pose->position.y - T[7];
        err[2] = pose->position.z - T[11];

        double norm = sqrt(err[0] * err[0] + err[1] * err[1] + err[2] * err[2]);

        if(norm < 1e-4) {
            array_to_joints(q, joints);
            return ARM_SUCCESS;
        }

        // 数值雅可比
        double J[36];
        double eps = 1e-6;

        for(int i = 0; i < 6; i++) {

            double q_tmp[6];
            memcpy(q_tmp, q, sizeof(q));

            q_tmp[i] += eps;

            double T2[16];
            fk_compute(q_tmp, T2);

            J[0 * 6 + i] = (T2[3] - T[3]) / eps;
            J[1 * 6 + i] = (T2[7] - T[7]) / eps;
            J[2 * 6 + i] = (T2[11] - T[11]) / eps;

            J[3 * 6 + i] = 0;
            J[4 * 6 + i] = 0;
            J[5 * 6 + i] = 0;
        }

        double JT[36];
        for(int i = 0; i < 6; i++)
            for(int j = 0; j < 6; j++)
                JT[i * 6 + j] = J[j * 6 + i];

        double JJT[36];
        mat6_mul(JT, J, JJT);

        // 阻尼
        for(int i = 0; i < 6; i++)
            JJT[i * 6 + i] += 0.01;

        double inv[36];
        if(mat6_inverse(JJT, inv) != 0)
            return ARM_ERROR_SINGULARITY;

        double tmp[36];
        mat6_mul(inv, JT, tmp);

        double dq[6];
        mat6_vec_mul(tmp, err, dq);

        for(int i = 0; i < 6; i++) {
            q[i] += dq[i];

            // 限位
            if(q[i] < arm_mdh.qmin[i]) q[i] = arm_mdh.qmin[i];
            if(q[i] > arm_mdh.qmax[i]) q[i] = arm_mdh.qmax[i];
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
static void joints_to_array(const SixDofJoint_t* j, double* q) {
    q[0] = j->joint_1;
    q[1] = j->joint_2;
    q[2] = j->joint_3;
    q[3] = j->joint_4;
    q[4] = j->joint_5;
    q[5] = j->joint_6;
}

/**
 * @brief 将关节角度数组转换回结构体形式
 * @param q 关节角度数组
 * @param j 输出的关节结构体
 */
static void array_to_joints(const double* q, SixDofJoint_t* j) {
    j->joint_1 = q[0];
    j->joint_2 = q[1];
    j->joint_3 = q[2];
    j->joint_4 = q[3];
    j->joint_5 = q[4];
    j->joint_6 = q[5];
}

/**
 * @brief 生成 4x4 单位矩阵
 * @param T 输出的单位矩阵
 */
static void mat4_identity(double* T) {
    memset(T, 0, sizeof(double) * 16);
    T[0] = T[5] = T[10] = T[15] = 1.0;
}

/**
 * @brief 计算两个 4x4 矩阵的乘积
 * @param A 输入矩阵 A
 * @param B 输入矩阵 B
 * @param R 输出矩阵 R = A * B
 */
static void mat4_mul(const double* A, const double* B, double* R) {
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            R[i * 4 + j] = 0;
            for(int k = 0; k < 4; k++) {
                R[i * 4 + j] += A[i * 4 + k] * B[k * 4 + j];
            }
        }
    }
}

/**
 * @brief 根据 MDH 参数生成变换矩阵
 * @param T 输出的 4x4 变换矩阵
 * @param alpha 关节 i-1 的扭转角
 * @param a 关节 i-1 的连杆长度
 * @param theta 关节 i 的关节角
 * @param d 关节 i 的连杆偏移
 */
static void get_tf_matrix(double* T, double alpha, double a, double theta, double d) {
    T[0] = cos(theta); T[1] = -sin(theta); T[2] = 0; T[3] = a;
    T[4] = sin(theta) * cos(alpha); T[5] = cos(theta) * cos(alpha); T[6] = -sin(alpha); T[7] = -d * sin(alpha);
    T[8] = sin(theta) * sin(alpha); T[9] = cos(theta) * sin(alpha); T[10] = cos(alpha); T[11] = d * cos(alpha);
    T[12] = 0; T[13] = 0; T[14] = 0; T[15] = 1;
}

/**
 * @brief 计算机械臂的正运动学，得到末端的变换矩阵
 * @param q 关节角度数组
 * @param T_out 输出的末端变换矩阵
 */
static void fk_compute(const double* q, double* T_out) {
    double T[16];
    mat4_identity(T);

    for(int i = 0; i < 6; i++) {
        double Ti[16];
        get_tf_matrix(Ti,
            arm_mdh.alpha[i],
            arm_mdh.a[i],
            q[i] + arm_mdh.offset[i],
            arm_mdh.d[i]);

        double tmp[16];
        mat4_mul(T, Ti, tmp);
        memcpy(T, tmp, sizeof(tmp));
    }

    memcpy(T_out, T, sizeof(double) * 16);
}

/**
 * @brief 将 4x4 变换矩阵转换为位姿结构体
 * @param T 输入的 4x4 变换矩阵
 * @param pose 输出的位姿结构体
 */
static void matrix_to_pose(const double* T, Pose_t* pose) {
    pose->position.x = T[3];
    pose->position.y = T[7];
    pose->position.z = T[11];

    double r11 = T[0], r22 = T[5], r33 = T[10];
    double qw = sqrt(1 + r11 + r22 + r33) / 2.0;

    pose->orientation.w = qw;
    pose->orientation.x = (T[9] - T[6]) / (4 * qw);
    pose->orientation.y = (T[2] - T[8]) / (4 * qw);
    pose->orientation.z = (T[4] - T[1]) / (4 * qw);
}

/**
 * @brief 生成 6x6 单位矩阵
 * @param A 输出的单位矩阵
 */
static void mat6_identity(double* A) {
    memset(A, 0, sizeof(double) * 36);
    for(int i = 0; i < 6; i++) A[i * 6 + i] = 1.0;
}

/**
 * @brief 计算两个 6x6 矩阵的乘积
 * @param A 输入矩阵 A
 * @param B 输入矩阵 B
 * @param R 输出矩阵 R = A * B
 */
static void mat6_mul(const double* A, const double* B, double* R) {
    for(int i = 0; i < 6; i++) {
        for(int j = 0; j < 6; j++) {
            R[i * 6 + j] = 0;
            for(int k = 0; k < 6; k++) {
                R[i * 6 + j] += A[i * 6 + k] * B[k * 6 + j];
            }
        }
    }
}

/**
 * @brief 计算 6x6 矩阵与 6x1 向量的乘积
 * @param A 输入矩阵 A
 * @param x 输入向量 x
 * @param y 输出向量 y = A * x
 */
static void mat6_vec_mul(const double* A, const double* x, double* y) {
    for(int i = 0; i < 6; i++) {
        y[i] = 0;
        for(int j = 0; j < 6; j++) {
            y[i] += A[i * 6 + j] * x[j];
        }
    }
}

/**
 * @brief 计算 6x6 矩阵的逆矩阵，使用高斯消元法
 * @param A 输入矩阵 A
 * @param inv 输出矩阵 inv = A^-1
 * @return int 0 成功，-1 矩阵不可逆
 */
static int mat6_inverse(double* A, double* inv) {
    mat6_identity(inv);

    for(int i = 0; i < 6; i++) {
        double pivot = A[i * 6 + i];
        if(fabs(pivot) < 1e-9) return -1;

        double inv_p = 1.0 / pivot;

        for(int j = 0; j < 6; j++) {
            A[i * 6 + j] *= inv_p;
            inv[i * 6 + j] *= inv_p;
        }

        for(int k = 0; k < 6; k++) {
            if(k == i) continue;

            double f = A[k * 6 + i];

            for(int j = 0; j < 6; j++) {
                A[k * 6 + j] -= f * A[i * 6 + j];
                inv[k * 6 + j] -= f * inv[i * 6 + j];
            }
        }
    }
    return 0;
}
