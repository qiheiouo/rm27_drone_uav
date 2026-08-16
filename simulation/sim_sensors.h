/*
 * sim_sensors.h - 仿真传感器（第二阶段）
 *
 * IMU：机体系比力（含重力）+ 陀螺真值 + 固定偏置 + 噪声。
 * VO：视觉里程计 = 真值 + 常值漂移速度 + 噪声 + 撞击后 blackout，
 *     模拟真实 VIO 的漂移特性（第三层定位存在的意义）。
 * 相机：针孔投影产生像素观测（含像素噪声/量化），
 *     目标检测算法端从像素 + 已知尺寸重建相对位置。
 *
 * 全部噪声由 seed 决定的 LCG 产生，仿真可复现。
 */
#ifndef SIM_SENSORS_H
#define SIM_SENSORS_H

#include <stdint.h>
#include "nav_math.h"
#include "state_estimator.h"
#include "impact_detector.h"
#include "camera.h"
#include "sim_dynamics.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t rng_state;

    /* IMU */
    Vec3f imu_accel_bias;    /* 固定偏置（每次上电不同 → 由 seed 决定） */
    Vec3f imu_gyro_bias;
    float imu_accel_noise;
    float imu_gyro_noise;

    /* VO */
    Vec3f vo_drift_vel;      /* 常值漂移速度（seed 决定） */
    float vo_yaw_drift_rate;
    Vec3f vo_drift;          /* 累计漂移 */
    float vo_yaw_drift;
    float vo_pos_noise;
    float vo_vel_noise;
    float vo_yaw_noise;

    /* 相机 */
    float pixel_noise;       /* 像素噪声 sigma (px) */
    float target_min_px;     /* 最小可检测像素尺寸（等效检测距离上限） */
    float home_min_px;
    float tof_noise;         /* ToF 测距噪声 sigma (m) */
} SimSensors;

void sim_sensors_init(SimSensors *sen, uint32_t seed);

/* IMU：机体系比力 + 陀螺 */
void sim_sensors_imu(SimSensors *sen, const SimState *truth, uint32_t t_ms, ImuSample *out);

/* VO：位置/速度/偏航 + 漂移；vision_freeze=1 时本帧无效（--sim-vo 对照路径） */
void sim_sensors_vo(SimSensors *sen, const SimState *truth,
                    uint8_t vision_freeze, float dt, uint32_t t_ms, OdomSample *out);

/* ToF 测距：沿机体 -z 到地面（z=0）的距离，含噪声。
 * 为光流提供独立于估计器的高度尺度（打破 scale 正反馈） */
float sim_sensors_tof(SimSensors *sen, const SimState *truth);

/* 目标像素观测（前视相机）；vision_freeze=1 模拟撞击后运动模糊 */
void sim_sensors_target(SimSensors *sen, const SimState *truth, const CameraModel *cam,
                        Vec3f target_pos, float target_size,
                        uint8_t vision_freeze, uint32_t t_ms, PixelObs *out);

/* 基座 marker 像素观测（下视相机） */
void sim_sensors_home(SimSensors *sen, const SimState *truth, const CameraModel *cam,
                      Vec3f home_pos, float marker_size,
                      uint8_t vision_freeze, uint32_t t_ms, PixelObs *out);

#ifdef __cplusplus
}
#endif

#endif /* SIM_SENSORS_H */
