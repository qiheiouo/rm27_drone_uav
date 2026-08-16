/*
 * home_detector.h - 基座标志检测 / 绝对定位恢复
 *
 * Plan 第 7 节第三层：基座自行设计，可放置高对比图案 / fiducial / LED。
 * 只要回到基座附近，即可通过 marker 重建绝对参考（relocalize + 精确停靠）。
 *
 * 第一阶段：观测量由仿真传感器给出。
 */
#ifndef HOME_DETECTOR_H
#define HOME_DETECTOR_H

#include <stdint.h>
#include "nav_math.h"
#include "lowpass.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  visible;
    Vec3f    rel_pos;        /* 基座 marker 相对机体位置 (m) */
    uint32_t timestamp_ms;
} HomeObs;

typedef struct {
    uint8_t visible;
    Vec3f   rel_pos;
    float   time_since_update;
} HomeTrack;

typedef struct {
    LowPass1 fx, fy, fz;
    float    lost_time;
    HomeTrack out;
} HomeDetector;

void home_detector_init(HomeDetector *det, float filter_cutoff_hz, float dt);
void home_detector_update(HomeDetector *det, const HomeObs *obs, float dt);

#ifdef __cplusplus
}
#endif

#endif /* HOME_DETECTOR_H */
