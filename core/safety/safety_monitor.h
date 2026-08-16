/*
 * safety_monitor.h - 任务级安全监控
 *
 * 负责：任务时间预算（30 s 规则）、地理围栏。
 * 估计器健康由 Mission FSM 直接观察 NavState.status 处理。
 */
#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <stdint.h>
#include "nav_math.h"
#include "state_estimator.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float  max_mission_time_s;   /* 规则限制：单次飞行 <= 30 s */
    float  geofence_radius_m;    /* 以 home 为圆心的水平围栏 */
    float  geofence_max_alt_m;
} SafetyConfig;

typedef struct {
    SafetyConfig cfg;
    float   elapsed_s;
    uint8_t time_exceeded;       /* 低电量/超时 → 触发提前返航 */
    uint8_t geofence_violation;  /* 出围栏 → 紧急稳定/降落 */
} SafetyMonitor;

void safety_init(SafetyMonitor *mon, const SafetyConfig *cfg);
void safety_update(SafetyMonitor *mon, const NavState *nav, const Vec3f *home, float dt);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_MONITOR_H */
