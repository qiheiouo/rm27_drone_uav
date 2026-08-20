/* Future-sampled inter-agent trajectory conflict detection. */
#ifndef COLLISION_INTERFACE_H
#define COLLISION_INTERFACE_H

#include <stdint.h>
#include "nav_math.h"
#include "agent_state.h"
#include "trajectory_message.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SWARM_RISK_NONE = 0,
    SWARM_RISK_WARNING,
    SWARM_RISK_CRITICAL
} SwarmRiskLevel;

typedef struct {
    uint8_t conflict;
    uint8_t other_agent_id;
    float min_separation;
    float time_to_conflict_s;
    float closing_speed_mps;
    SwarmRiskLevel risk;
} CollisionReport;

typedef struct {
    float safe_separation_m;
    float critical_separation_m;
    float prediction_horizon_s;
    float sample_dt_s;
    float max_message_age_s;
} CollisionConfig;

typedef struct {
    AgentState state;
    TrajectoryMessage trajectory;
} OtherAgentPrediction;

void collision_init(CollisionConfig *cfg, float safe_separation_m);
CollisionReport collision_check(const CollisionConfig *cfg,
                                const Vec3f *self_points, uint8_t self_point_count,
                                const SwarmView *swarm);
CollisionReport collision_check_trajectories(
    const CollisionConfig *cfg,
    const TrajectoryMessage *own_trajectory,
    const OtherAgentPrediction *others,
    uint8_t other_count,
    uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* COLLISION_INTERFACE_H */
