/*
 * pos_controller.h - 导航层 → 飞控层 的控制接口
 *
 * 导航/制导输出 GuidanceOutput（位置/速度/偏航目标），
 * 本模块将其转换为加速度指令（真实硬件上对应姿态/推力目标，
 * 由低层姿态环执行）。仿真中直接驱动点质量动力学。
 */
#ifndef POS_CONTROLLER_H
#define POS_CONTROLLER_H

#include <stdint.h>
#include "nav_math.h"
#include "state_estimator.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 制导层统一输出 */
typedef struct {
    Vec3f   pos_sp;      /* 位置目标 (m)，use_pos_sp=1 时有效 */
    Vec3f   vel_sp;      /* 速度目标/前馈 (m/s) */
    float   yaw_sp;      /* 偏航目标 (rad) */
    uint8_t use_pos_sp;  /* 1 = 位置+速度模式，0 = 纯速度模式 */
} GuidanceOutput;

/* 控制器输出（交给低层飞控 / 仿真动力学） */
typedef struct {
    Vec3f accel_cmd;     /* 期望加速度 (m/s^2) */
    float yaw_rate_cmd;  /* 期望偏航角速度 (rad/s) */
} CtrlOutput;

typedef struct {
    float kp_pos;        /* 位置环增益 */
    float kp_vel;        /* 速度环增益 */
    float max_vel;       /* 速度上限 (m/s) */
    float max_accel;     /* 加速度上限 (m/s^2) */
    float kp_yaw;
    float max_yaw_rate;
} PosCtrlParams;

void pos_controller_update(const PosCtrlParams *params,
                           const GuidanceOutput *sp,
                           const NavState *nav,
                           CtrlOutput *out);

#ifdef __cplusplus
}
#endif

#endif /* POS_CONTROLLER_H */
