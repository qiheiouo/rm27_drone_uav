/*
 * trajectory.h - 多项式轨迹（MINCO-lite 的低保底版本）
 *
 * Plan 第 10 节：第一版不做完整 MINCO 时空联合优化。
 * 本模块是中间档：航点间 minimum-jerk 五次多项式 + 简单时间分配
 * （按巡航速度缩放段时长），端点速度/加速度为零，段间 C2 连续。
 * 固定维度、闭式系数、无迭代优化，MCU 可运行。
 *
 * 用法：
 *   traj_build(&traj, start_pos, start_vel, waypoints, ...);
 *   每 tick: traj_evaluate(&traj, t, &pos, &vel, &accel) → 制导前馈
 */
#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdint.h>
#include "nav_math.h"
#include "waypoint.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TRAJ_MAX_SEGS WP_QUEUE_MAX

/* 单轴五次多项式 p(t) = c0 + c1 t + ... + c5 t^5 */
typedef struct {
    float c[6];
} Poly5;

typedef struct {
    Poly5 px, py, pz;
    float dur;          /* 段时长 (s) */
} PolySegment;

typedef struct {
    PolySegment segs[TRAJ_MAX_SEGS];
    uint8_t     count;
    float       duration;   /* 总时长 (s) */
} Trajectory;

/*
 * 由航点队列构建轨迹。start_vel 为当前速度（平滑接入），
 * 各航点速度/加速度为 0（stop-and-go，简单可靠）。
 * 段时间分配：T = time_scale * dist / waypoint_speed，
 * min-jerk 峰值速度 ≈ 1.875*dist/T ≈ (1.875/time_scale)*speed。
 */
void traj_build(Trajectory *traj, Vec3f start_pos, Vec3f start_vel,
                const WaypointQueue *route, float time_scale);

/* 单航点版本（返航/撤离用） */
void traj_build_single(Trajectory *traj, Vec3f start_pos, Vec3f start_vel,
                       Vec3f goal_pos, float speed, float time_scale);

/* t 为从构建时刻起的时间 (s)，自动 clamp 到 [0, duration] */
void traj_evaluate(const Trajectory *traj, float t,
                   Vec3f *pos, Vec3f *vel, Vec3f *accel);

uint8_t traj_done(const Trajectory *traj, float t);

#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_H */
