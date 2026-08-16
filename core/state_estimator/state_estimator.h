/*
 * state_estimator.h - 状态估计（第二阶段：IMU 惯导 + 视觉里程计融合）
 *
 * 结构：
 *   IMU(陀螺/加速度计) → INS 预测（姿态积分 + 比力积分）
 *   Mahony 姿态修正（加速度计倾斜校正 + 陀螺偏置学习）
 *   VO(OdomSample) → 互补校正（位置/速度/偏航）
 *   VO 有效性/撞击事件 → 健康状态机
 *
 * 健康状态（Plan 第 6 节，不假设 VIO 永远连续）：
 *   TRACKING / DEGRADED / LOST / RECOVERING / RELOCALIZED
 *
 * EST_MODE_TRUTH 保留第一阶段的透传模式（单元测试/对照实验用）。
 */
#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <stdint.h>
#include "nav_math.h"
#include "impact_detector.h"   /* ImuSample */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EST_TRACKING = 0,   /* 正常跟踪（VO 有效） */
    EST_DEGRADED,       /* 退化：VO 短暂失效/撞击后，纯 IMU 推算 */
    EST_LOST,           /* 丢失：输出冻结，禁止大范围机动 */
    EST_RECOVERING,     /* 恢复中：VO 重新有效，收敛中 */
    EST_RELOCALIZED     /* 已重定位（一个 tick 后回到 TRACKING） */
} EstStatus;

typedef enum {
    EST_MODE_TRUTH = 0, /* 透传里程计（第一阶段行为/对照） */
    EST_MODE_INS        /* IMU 预测 + VO 校正 */
} EstimatorMode;

/* 导航状态输出 */
typedef struct {
    Vec3f    pos;          /* (m) */
    Vec3f    vel;          /* (m/s) */
    float    yaw;          /* (rad) */
    float    yaw_rate;     /* (rad/s) */
    Quatf    att;          /* 姿态（机体→导航系），相机重建等使用 */
    EstStatus status;
    uint32_t timestamp_ms;
} NavState;

/* 视觉里程计输入（仿真 VO / 未来 VIO 输出） */
typedef struct {
    Vec3f    pos;
    Vec3f    vel;
    float    yaw;
    float    yaw_rate;
    Quatf    att;          /* VO 姿态输出（真实 VIO 均提供姿态） */
    uint8_t  valid;        /* 0 = 本帧视觉无效（特征骤减/模糊） */
    uint32_t timestamp_ms;
} OdomSample;

typedef struct {
    EstimatorMode mode;
    /* 健康机 */
    float  vo_degraded_after_s;  /* VO 失效多久 → DEGRADED */
    float  vo_lost_after_s;      /* VO 失效多久 → LOST */
    float  recovering_hold_s;    /* VO 恢复后收敛时间 → RELOCALIZED */
    float  lost_timeout_s;       /* LOST 最长盲等（之后尝试纯 IMU 恢复） */
    float  impact_blind_s;       /* 撞击后拒绝视觉输入的时长（视觉冻结） */
    uint8_t lost_on_impact;      /* 1 = 撞击直接 LOST（最坏情况演练） */
    /* Mahony 姿态修正 */
    float  kp_tilt;              /* 加速度计倾斜校正增益 */
    float  ki_gyro_bias;         /* 陀螺偏置学习增益 */
    /* VO 互补校正增益 (1/s) */
    float  kp_vo_pos;
    float  kp_vo_vel;
    float  kp_vo_yaw;
} EstimatorConfig;

typedef struct {
    EstimatorConfig cfg;
    NavState  out;
    float     timer;             /* 当前健康状态持续时间 */
    float     vo_invalid_time;   /* VO 连续失效时长 */
    float     blind_time;        /* 撞击后视觉冻结剩余时间 */
    NavState  last_valid;        /* LOST 时冻结的输出 */
    /* INS 内部状态 */
    Vec3f     ins_pos;
    Vec3f     ins_vel;
    Quatf     ins_att;
    Vec3f     gyro_bias;
    Vec3f     accel_bias;
} StateEstimator;

void estimator_init(StateEstimator *est, const EstimatorConfig *cfg);
/* 撞击事件通知（由 impact detector 触发） */
void estimator_notify_impact(StateEstimator *est);
/* 外部绝对参考注入（基座 marker 重建 → 第三层定位） */
void estimator_notify_relocalized(StateEstimator *est, const NavState *absolute_ref);
/* 每 tick：imu 始终有效；vo->valid 表示视觉是否可用 */
void estimator_update(StateEstimator *est, const ImuSample *imu,
                      const OdomSample *vo, float dt);

const char *est_status_name(EstStatus s);

#ifdef __cplusplus
}
#endif

#endif /* STATE_ESTIMATOR_H */
