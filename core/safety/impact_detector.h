/*
 * impact_detector.h - 接触/撞击检测
 *
 * Plan 第 8 节：接触目标必须独立成状态模块，第一步是可靠检测。
 * 第一阶段使用加速度计模长尖峰 + 陀螺尖峰的组合判定，
 * 后续可加入 motor response / attitude error 通道。
 */
#ifndef IMPACT_DETECTOR_H
#define IMPACT_DETECTOR_H

#include <stdint.h>
#include "nav_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* IMU 采样（机体系，仿真第一阶段直接用导航系近似） */
typedef struct {
    Vec3f    accel;      /* 比力/加速度 (m/s^2) */
    Vec3f    gyro;       /* 角速度 (rad/s) */
    uint32_t timestamp_ms;
} ImuSample;

typedef struct {
    float   accel_spike_threshold;  /* |accel| 偏离 1g 超过该值视为候选撞击 (m/s^2) */
    float   gyro_spike_threshold;   /* |gyro| 超过该值视为候选撞击 (rad/s) */
    uint8_t confirm_samples;        /* 连续确认帧数（抗振动误检） */
} ImpactDetectorConfig;

typedef struct {
    ImpactDetectorConfig cfg;
    uint8_t triggered;          /* 锁存：直到 reset */
    uint8_t confirm_count;
} ImpactDetector;

void impact_detector_init(ImpactDetector *det, const ImpactDetectorConfig *cfg);
void impact_detector_reset(ImpactDetector *det);
/* 返回 1 = 本帧确认新撞击（上升沿）；det->triggered 保持锁存 */
uint8_t impact_detector_update(ImpactDetector *det, const ImuSample *sample);

#ifdef __cplusplus
}
#endif

#endif /* IMPACT_DETECTOR_H */
