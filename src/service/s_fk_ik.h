#ifndef _s_fk_ik_h_
#define _s_fk_ik_h_

#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

#define M_PI 3.14159265358979323846f
#define M_2PI (2.0f * M_PI)

typedef enum {
    ARM_SUCCESS = 0,
    ARM_ERROR = 0,
    ARM_ERROR_INVALID_JOINTS,
    ARM_ERROR_INVALID_POSE,
    ARM_ERROR_SINGULARITY,
    ARM_ERROR_OUT_OF_REACH
} ArmErrorCode_t;

typedef struct {
    double alpha[6];
    double a[6];
    double d[6];
    double offset[6];
    double qmin[6];
    double qmax[6];
} ArmMDH_t;

typedef struct {
    double x;
    double y;
    double z;
} Point_t;

typedef struct {
    double x;
    double y;
    double z;
    double w;
} Quaternion_t;

typedef struct {
    Point_t position;
    Quaternion_t orientation;
} Pose_t;

typedef struct {
    Pose_t pose;
    Point_t point;
    Quaternion_t quat;
} geometry_msgs_t;

typedef struct {
    double joint_1;
    double joint_2;
    double joint_3;
    double joint_4;
    double joint_5;
    double joint_6;
} SixDofJoint_t;

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
