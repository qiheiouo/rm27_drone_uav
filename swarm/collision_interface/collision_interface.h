/*
 * collision_interface.h - 动态障碍/轨迹冲突检测接口
 *
 * 第一版：other_count = 0 时恒为无冲突，几乎零成本。
 * 未来：将其他无人机的共享轨迹视为动态障碍，
 * 做 trajectory conflict detection + reciprocal avoidance。
 */
#ifndef COLLISION_INTERFACE_H
#define COLLISION_INTERFACE_H

#include <stdint.h>
#include "nav_math.h"
#include "agent_state.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t conflict;           /* 1 = 预测存在冲突 */
    uint8_t other_agent_id;     /* 冲突对象 */
    float   min_separation;     /* 预测最小间距 (m) */
} CollisionReport;

typedef struct {
    float safe_separation_m;    /* 安全间距 */
} CollisionConfig;

void collision_init(CollisionConfig *cfg, float safe_separation_m);

/*
 * 检查自机预测轨迹（points/dt 定义的折线）与他机当前状态的冲突。
 * self_points 可为 0（count=0），表示只检查当前位置。
 */
CollisionReport collision_check(const CollisionConfig *cfg,
                                const Vec3f *self_points, uint8_t self_point_count,
                                const SwarmView *swarm);

#ifdef __cplusplus
}
#endif

#endif /* COLLISION_INTERFACE_H */
