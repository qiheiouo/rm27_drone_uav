/* Lightweight deterministic yielding policy layered after conflict detection. */
#ifndef SWARM_AVOIDANCE_H
#define SWARM_AVOIDANCE_H

#include <stdint.h>
#include "collision_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SWARM_DISABLED = 0,
    SWARM_ENABLED
} SwarmMode;

typedef struct {
    SwarmMode mode;
    float lateral_bias_mps;
    float critical_climb_mps;
    float time_shift_s;
} SwarmAvoidanceConfig;

typedef struct {
    Vec3f velocity_bias;
    float requested_time_shift_s;
    uint8_t yielding;
    uint8_t active;
} SwarmAvoidanceDecision;

void swarm_avoidance_default_config(SwarmAvoidanceConfig *cfg);
SwarmAvoidanceDecision swarm_avoidance_decide(
    const SwarmAvoidanceConfig *cfg,
    uint8_t self_agent_id,
    const CollisionReport *report,
    Vec3f desired_velocity);

#ifdef __cplusplus
}
#endif

#endif /* SWARM_AVOIDANCE_H */
