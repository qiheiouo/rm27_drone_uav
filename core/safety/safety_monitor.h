/* Hierarchical safety monitor. It proposes overrides; the mission FSM owns transitions. */
#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <stdint.h>
#include "nav_math.h"
#include "state_estimator.h"
#include "impact_detector.h"
#include "obstacle_avoidance.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SAFETY_SAFE = 0,
    SAFETY_DEGRADED,
    SAFETY_RETURN_REQUIRED,
    SAFETY_RECOVERY_REQUIRED,
    SAFETY_EMERGENCY
} SafetyLevel;

enum {
    SAFETY_REASON_NONE = 0u,
    SAFETY_REASON_SOFT_DEADLINE = 1u << 0,
    SAFETY_REASON_HARD_DEADLINE = 1u << 1,
    SAFETY_REASON_GEOFENCE = 1u << 2,
    SAFETY_REASON_ESTIMATOR = 1u << 3,
    SAFETY_REASON_IMPACT = 1u << 4,
    SAFETY_REASON_COLLISION = 1u << 5,
    SAFETY_REASON_TRAJECTORY = 1u << 6,
    SAFETY_REASON_CONTROLLER = 1u << 7,
    SAFETY_REASON_SWARM_LINK = 1u << 8
};

typedef struct {
    float max_mission_time_s;
    float soft_return_deadline_s;
    float hard_return_deadline_s;
    float geofence_radius_m;
    float geofence_min_alt_m;
    float geofence_max_alt_m;
    float min_estimator_quality;
    float controller_saturation_timeout_s;
    float collision_critical_timeout_s;
    float trajectory_invalid_timeout_s;
} SafetyConfig;

typedef struct {
    const NavState *nav;
    Vec3f home_position;
    ImpactState impact_state;
    CollisionRiskLevel collision_risk;
    uint8_t trajectory_valid;
    uint8_t controller_saturated;
    uint8_t swarm_link_guard_active;
    float dt;
} SafetyInput;

typedef struct {
    SafetyLevel level;
    uint32_t reason_mask;
    float elapsed_s;
    float remaining_s;
    uint8_t request_return;
    uint8_t request_recovery;
    uint8_t request_emergency;
} SafetyDecision;

typedef struct {
    SafetyConfig cfg;
    float elapsed_s;
    float remaining_s;
    float controller_saturation_time_s;
    float collision_critical_time_s;
    float trajectory_invalid_time_s;
    uint8_t time_exceeded;
    uint8_t soft_return_reached;
    uint8_t hard_return_reached;
    uint8_t geofence_violation;
    SafetyDecision decision;
} SafetyMonitor;

void safety_init(SafetyMonitor *monitor, const SafetyConfig *cfg);
void safety_update(SafetyMonitor *monitor, const NavState *nav,
                   const Vec3f *home, float dt);
SafetyDecision safety_update_full(SafetyMonitor *monitor,
                                  const SafetyInput *input);
const char *safety_level_name(SafetyLevel level);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_MONITOR_H */
