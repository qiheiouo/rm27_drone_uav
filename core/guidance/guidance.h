/*
 * guidance.h - 制导律集合
 *
 * 每个函数都是：输入结构体 → 输出 GuidanceOutput（位置/速度/偏航目标）。
 * 对应 Plan 第 12 节的 nav 纯函数风格：
 *   cruise_guidance_update / target_guidance_update /
 *   home_guidance_update / recovery_guidance_update
 */
#ifndef GUIDANCE_H
#define GUIDANCE_H

#include <stdint.h>
#include "nav_math.h"
#include "state_estimator.h"
#include "target_tracker.h"
#include "home_detector.h"
#include "waypoint.h"
#include "pos_controller.h"   /* GuidanceOutput 定义 */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float approach_speed;   /* 接近速度上限 (m/s) */
    float kp;               /* 相对位置 → 速度 增益 */
    float min_closing_speed;/* 最小接近速度，防止末端停滞 (m/s) */
} TerminalParams;

typedef struct {
    float search_alt;        /* 未见到 marker 时的搜索高度 (m) */
    float descend_speed;     /* 停靠下降速度 (m/s) */
    float kp_lateral;        /* 横向伺服增益 */
    float max_lateral_speed;
    float lateral_tol;       /* 横向对准容差，对准后才开始下降 (m) */
    float spiral_rate;       /* 螺旋搜索半径扩张速率 (m/s) */
    float spiral_max_radius; /* 螺旋搜索最大半径 (m) */
    float spiral_omega;      /* 螺旋角速度 (rad/s) */
    float blind_land_alt;    /* 低于该高度且 marker 短暂丢失时垂直盲降 (m) */
    float blind_land_timeout;/* 盲降允许的最长 marker 丢失时间 (s) */
} HomeParams;

/* 起飞/悬停：位置模式 */
void guidance_takeoff(const Vec3f *target_pos, const NavState *nav, GuidanceOutput *out);
void guidance_hold(const Vec3f *hold_pos, const NavState *nav, GuidanceOutput *out);

/* 巡航/搜索/返航：朝航点飞行，yaw 朝运动方向 */
void cruise_guidance_update(const Waypoint *wp, const NavState *nav, GuidanceOutput *out);

/* 末端制导：目标相对视觉伺服（第二层定位，脱离全局坐标） */
void target_guidance_update(const TargetTrack *target, const NavState *nav,
                            const TerminalParams *params, GuidanceOutput *out);

/* 撞击恢复：速度阻尼（姿态稳定由低层飞控负责） */
void recovery_guidance_update(const NavState *nav, float damping_gain, GuidanceOutput *out);

/* 基座制导：marker 可见 → 视觉伺服下降；不可见 → 到估计 home 上方搜索高度 */
void home_guidance_update(const HomeTrack *home, const NavState *nav,
                          const Vec3f *home_est, const HomeParams *params,
                          GuidanceOutput *out);

/* 紧急降落：垂直慢速下降 */
void guidance_land(const NavState *nav, float descend_speed, GuidanceOutput *out);

#ifdef __cplusplus
}
#endif

#endif /* GUIDANCE_H */
