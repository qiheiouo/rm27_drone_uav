/*
 * agent_state.h - 多机状态抽象（Plan 第 4 节）
 *
 * 第一版：other_count = 0，不支付多机计算成本。
 * 数据结构与接口从一开始就保留，未来并行飞行时：
 *   other_count > 0 → 其他无人机视为动态障碍 → 轨迹冲突检测 → 互惠避让。
 */
#ifndef AGENT_STATE_H
#define AGENT_STATE_H

#include <stdint.h>
#include "nav_math.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SWARM_MAX_OTHER_AGENTS 4u

typedef struct {
    uint8_t  agent_id;
    uint32_t timestamp_ms;
    Vec3f    pos;
    Vec3f    vel;
} AgentState;

typedef struct {
    AgentState self;
    AgentState others[SWARM_MAX_OTHER_AGENTS];
    uint8_t    other_count;   /* 第一版恒为 0 */
} SwarmView;

static inline void swarm_view_init(SwarmView *view, uint8_t self_id)
{
    uint8_t i;
    view->self.agent_id = self_id;
    view->self.timestamp_ms = 0u;
    view->self.pos = vec3_zero();
    view->self.vel = vec3_zero();
    for (i = 0u; i < SWARM_MAX_OTHER_AGENTS; i++) {
        view->others[i].agent_id = 0u;
        view->others[i].timestamp_ms = 0u;
        view->others[i].pos = vec3_zero();
        view->others[i].vel = vec3_zero();
    }
    view->other_count = 0u;
}

#ifdef __cplusplus
}
#endif

#endif /* AGENT_STATE_H */
