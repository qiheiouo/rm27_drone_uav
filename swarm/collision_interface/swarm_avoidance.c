#include "swarm_avoidance.h"

void swarm_avoidance_default_config(SwarmAvoidanceConfig *cfg)
{
    cfg->mode = SWARM_DISABLED;
    cfg->lateral_bias_mps = 0.5f;
    cfg->critical_climb_mps = 0.35f;
    cfg->time_shift_s = 0.4f;
}

SwarmAvoidanceDecision swarm_avoidance_decide(
    const SwarmAvoidanceConfig *cfg,
    uint8_t self_agent_id,
    const CollisionReport *report,
    Vec3f desired_velocity)
{
    SwarmAvoidanceDecision decision;
    decision.velocity_bias = vec3_zero();
    decision.requested_time_shift_s = 0.0f;
    decision.yielding = 0u;
    decision.active = 0u;

    if (cfg->mode == SWARM_DISABLED || !report->conflict) {
        return decision;
    }

    decision.active = 1u;
    decision.yielding = (self_agent_id > report->other_agent_id) ? 1u : 0u;
    if (decision.yielding) {
        Vec3f lateral = vec3_normalize_or(vec3(-desired_velocity.y,
                                                desired_velocity.x, 0.0f),
                                           vec3(0.0f, 1.0f, 0.0f));
        decision.velocity_bias = vec3_scale(lateral, cfg->lateral_bias_mps);
        decision.requested_time_shift_s = cfg->time_shift_s;
        if (report->risk == SWARM_RISK_CRITICAL) {
            decision.velocity_bias.z += cfg->critical_climb_mps;
        }
    }
    return decision;
}
