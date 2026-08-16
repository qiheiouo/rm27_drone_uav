/*
 * sim_dynamics.h - 仿真飞行动力学（第二阶段：点质量 + 姿态运动学）
 *
 * 平移仍为点质量（低层飞控理想跟随导航层加速度指令）。
 * 姿态采用推力矢量模型：机体 z 轴对齐比力方向（多旋翼物理特性），
 * 偏航跟踪指令；由此给出姿态四元数与机体系角速度真值，供 IMU 仿真。
 *
 * 撞击以瞬时速度/偏航/倾角扰动注入。
 */
#ifndef SIM_DYNAMICS_H
#define SIM_DYNAMICS_H

#include <stdint.h>
#include "nav_math.h"
#include "pos_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    Vec3f pos;
    Vec3f vel;
    float yaw;
    float yaw_rate;
    Vec3f last_accel;        /* 本步实际惯性加速度（含撞击），导航系 */
    Quatf att;               /* 姿态真值（机体 → 导航系） */
    Vec3f gyro_body;         /* 机体系角速度真值 (rad/s) */
    Vec3f spec_force_body;   /* 比力真值（机体系，含重力），加速度计量测 */
    Quatf att_prev;          /* 上一步姿态（陀螺真值差分用） */
} SimState;

typedef struct {
    float drag;          /* 线性阻尼系数 (1/s) */
    float max_speed;     /* (m/s) */
    float max_yaw_rate;  /* (rad/s) */
    float max_att_rate;  /* 姿态角速度上限 (rad/s)，模拟姿态环带宽 */
} SimParams;

/* 撞击扰动：active=1 时在本步注入 */
typedef struct {
    uint8_t active;
    Vec3f delta_v;       /* 瞬时速度改变 (m/s) */
    float delta_yaw;     /* 瞬时偏航扰动 (rad) */
    float delta_pitch;   /* 瞬时俯仰扰动 (rad) */
    float delta_roll;    /* 瞬时滚转扰动 (rad) */
} SimImpact;

void sim_dynamics_init(SimState *st, Vec3f pos0, float yaw0);
void sim_dynamics_step(SimState *st, const CtrlOutput *ctrl,
                       const SimParams *params, const SimImpact *impact, float dt);

#ifdef __cplusplus
}
#endif

#endif /* SIM_DYNAMICS_H */
