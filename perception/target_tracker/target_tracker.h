/*
 * target_tracker.h - 目标相对定位/跟踪
 *
 * Plan 第 7 节第二层：发现目标后尽量脱离全局坐标，
 * 直接使用相对量（rel x/y/z 或图像误差）做视觉伺服。
 *
 * 第一阶段：观测量由仿真传感器给出（相对位置 + 可见性）。
 * 后续由 camera target detector 提供同样结构的观测量。
 */
#ifndef TARGET_TRACKER_H
#define TARGET_TRACKER_H

#include <stdint.h>
#include "nav_math.h"
#include "lowpass.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 单帧目标观测（导航系下的相对位置；未来可换成图像误差 ex/ey/scale） */
typedef struct {
    uint8_t  visible;
    Vec3f    rel_pos;        /* 目标相对机体位置 (m)，visible=1 时有效 */
    uint32_t timestamp_ms;
} TargetObs;

/* 跟踪输出 */
typedef struct {
    uint8_t visible;             /* 当前帧可见 */
    Vec3f   rel_pos;             /* 滤波后的相对位置 */
    float   range;               /* 距离 (m) */
    float   time_since_update;   /* 距上次有效观测的时间 (s) */
} TargetTrack;

typedef struct {
    LowPass1 fx, fy, fz;    /* 相对位置平滑 */
    float    lost_time;
    TargetTrack out;
} TargetTracker;

void target_tracker_init(TargetTracker *trk, float filter_cutoff_hz, float dt);
void target_tracker_update(TargetTracker *trk, const TargetObs *obs, float dt);
void target_tracker_reset(TargetTracker *trk);

#ifdef __cplusplus
}
#endif

#endif /* TARGET_TRACKER_H */
