#ifndef _s_fk_ik_h_
#define _s_fk_ik_h_

#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/// @brief PI 常量
#define M_PI 3.14159265358979323846f
/// @brief 2PI 常量
#define M_2PI (2.0f * M_PI)

/**
 * @brief 机械臂错误码
 * @param ARM_SUCCESS 成功
 * @param ARM_ERROR 一般错误
 * @param ARM_ERROR_INVALID_JOINTS 无效的关节输入
 * @param ARM_ERROR_INVALID_POSE 无效的位姿输入
 * @param ARM_ERROR_SINGULARITY 奇异点
 * @param ARM_ERROR_OUT_OF_REACH 目标位姿不可达
 */
typedef enum {
    ARM_SUCCESS = 0,
    ARM_ERROR,
    ARM_ERROR_INVALID_JOINTS,
    ARM_ERROR_INVALID_POSE,
    ARM_ERROR_SINGULARITY,
    ARM_ERROR_OUT_OF_REACH
} ArmErrorCode_t;

/**
 * @brief 机械臂的MDH参数结构体
 * @param alpha 连杆扭转角
 * @param a 连杆长度
 * @param d 连杆偏距
 * @param offset 关节角偏移
 * @param qmin 关节最小角度
 * @param qmax 关节最大角度
 */
typedef struct {
    float alpha[6];
    float a[6];
    float d[6];
    float offset[6];
    float qmin[6];
    float qmax[6];
} ArmMDH_t;

/**
 * @brief 位置结构体
 * @param x x 坐标
 * @param y y 坐标
 * @param z z 坐标
 */
typedef struct {
    float x;
    float y;
    float z;
} Point_t;

/**
 * @brief 四元数结构体
 * @param w 实部
 * @param x 虚部 x
 * @param y 虚部 y
 * @param z 虚部 z
 */
typedef struct {
    float x;
    float y;
    float z;
    float w;
} Quaternion_t;

/**
 * @brief 位姿结构体
 * @param position 位置
 * @param orientation 姿态（四元数）
 */
typedef struct {
    Point_t position;
    Quaternion_t orientation;
} Pose_t;

/**
 * @brief geometry_msgs 结构体，包含位姿、位置和四元数
 * @param pose 位姿
 * @param point 位置
 * @param quat 四元数
 */
typedef struct {
    Pose_t pose;
    Point_t point;
    Quaternion_t quat;
} geometry_msgs_t;

/**
 * @brief 机械臂的六自由度关节结构体
 * @param joint_1 关节 1 角度
 * @param joint_2 关节 2 角度
 * @param joint_3 关节 3 角度
 * @param joint_4 关节 4 角度
 * @param joint_5 关节 5 角度
 * @param joint_6 关节 6 角度
 */
typedef struct {
    float joint_1;
    float joint_2;
    float joint_3;
    float joint_4;
    float joint_5;
    float joint_6;
} SixDofJoint_t;

/**
 * @brief 机械臂的六自由度关节解结构体，包含所有可能的解
 * @param num_solutions 解的数量
 * @param solution_1 解 1
 * @param solution_2 解 2
 * @param solution_3 解 3
 * @param solution_4 解 4
 * @param solution_5 解 5
 * @param solution_6 解 6
 * @param solution_7 解 7
 * @param solution_8 解 8
 */
typedef struct {
    uint8_t num_solutions;
    SixDofJoint_t solution_1;
    SixDofJoint_t solution_2;
    SixDofJoint_t solution_3;
    SixDofJoint_t solution_4;
    SixDofJoint_t solution_5;
    SixDofJoint_t solution_6;
    SixDofJoint_t solution_7;
    SixDofJoint_t solution_8;
} SixDofJointAll_t;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

ArmErrorCode_t s_six_dof_init(const ArmMDH_t* mdh);
ArmErrorCode_t s_six_dof_fk(const SixDofJoint_t* joints, Pose_t* pose);
ArmErrorCode_t s_six_dof_ik(const Pose_t* pose, SixDofJoint_t* joints, const SixDofJoint_t* current_joints);
ArmErrorCode_t s_six_dof_all_ik(const Pose_t* pose, SixDofJointAll_t* joints);

#endif
