#ifndef _s_fk_ik_h_
#define _s_fk_ik_h_



// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

typedef enum {
    ARM_SUCCESS = 0,
    ARM_ERROR_INVALID_JOINTS,
    ARM_ERROR_INVALID_POSE,
    ARM_ERROR_SINGULARITY,
    ARM_ERROR_OUT_OF_REACH
} ArmErrorCode_t;

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

// ! ========================= 接 口 函 数 声 明 ========================= ! //

ArmErrorCode_t s_six_dof_fk(const SixDofJoint_t* joints, Pose_t* pose);
ArmErrorCode_t s_six_dof_ik(const Pose_t* pose, SixDofJoint_t* joints);

#endif
