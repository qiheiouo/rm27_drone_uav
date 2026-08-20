#include "collision_interface.h"

static CollisionReport empty_report(void)
{
    CollisionReport report;
    report.conflict = 0u;
    report.other_agent_id = 0u;
    report.min_separation = 1e9f;
    report.time_to_conflict_s = 1e9f;
    report.closing_speed_mps = 0.0f;
    report.risk = SWARM_RISK_NONE;
    return report;
}

void collision_init(CollisionConfig *cfg, float safe_separation_m)
{
    cfg->safe_separation_m = safe_separation_m;
    cfg->critical_separation_m = 0.6f * safe_separation_m;
    cfg->prediction_horizon_s = 2.0f;
    cfg->sample_dt_s = 0.1f;
    cfg->max_message_age_s = 0.5f;
}

CollisionReport collision_check(const CollisionConfig *cfg,
                                const Vec3f *self_points, uint8_t self_point_count,
                                const SwarmView *swarm)
{
    CollisionReport report = empty_report();
    uint8_t other_index;
    uint8_t point_index;

    for (other_index = 0u; other_index < swarm->other_count &&
         other_index < SWARM_MAX_OTHER_AGENTS; other_index++) {
        const AgentState *other = &swarm->others[other_index];
        if (!other->valid || other->age_s > cfg->max_message_age_s) {
            continue;
        }
        if (self_point_count == 0u) {
            float distance = vec3_dist(swarm->self.pos, other->pos);
            if (distance < report.min_separation) {
                report.min_separation = distance;
                report.other_agent_id = other->agent_id;
            }
        }
        for (point_index = 0u; point_index < self_point_count; point_index++) {
            float future_time = (float)point_index * cfg->sample_dt_s;
            Vec3f other_position = vec3_add(other->pos,
                                            vec3_scale(other->vel, future_time));
            float distance = vec3_dist(self_points[point_index], other_position);
            if (distance < report.min_separation) {
                report.min_separation = distance;
                report.time_to_conflict_s = future_time;
                report.other_agent_id = other->agent_id;
                report.closing_speed_mps = vec3_norm(vec3_sub(swarm->self.vel, other->vel));
            }
        }
    }

    if (report.min_separation < cfg->critical_separation_m) {
        report.conflict = 1u;
        report.risk = SWARM_RISK_CRITICAL;
    } else if (report.min_separation < cfg->safe_separation_m) {
        report.conflict = 1u;
        report.risk = SWARM_RISK_WARNING;
    }
    return report;
}

CollisionReport collision_check_trajectories(
    const CollisionConfig *cfg,
    const TrajectoryMessage *own_trajectory,
    const OtherAgentPrediction *others,
    uint8_t other_count,
    uint32_t now_ms)
{
    CollisionReport report = empty_report();
    float time;
    uint8_t other_index;
    for (other_index = 0u; other_index < other_count &&
         other_index < SWARM_MAX_OTHER_AGENTS; other_index++) {
        const OtherAgentPrediction *other = &others[other_index];
        if (!other->state.valid || other->state.age_s > cfg->max_message_age_s ||
            !trajectory_message_is_fresh(&other->trajectory, now_ms)) {
            continue;
        }
        for (time = 0.0f; time <= cfg->prediction_horizon_s; time += cfg->sample_dt_s) {
            Vec3f own_position, own_velocity, other_position, other_velocity;
            float distance;
            if (!trajectory_message_evaluate(own_trajectory, time,
                                             &own_position, &own_velocity) ||
                !trajectory_message_evaluate(&other->trajectory, time,
                                             &other_position, &other_velocity)) {
                break;
            }
            distance = vec3_dist(own_position, other_position);
            if (distance < report.min_separation) {
                report.min_separation = distance;
                report.time_to_conflict_s = time;
                report.other_agent_id = other->state.agent_id;
                report.closing_speed_mps = vec3_norm(vec3_sub(own_velocity, other_velocity));
            }
        }
    }
    if (report.min_separation < cfg->critical_separation_m) {
        report.conflict = 1u;
        report.risk = SWARM_RISK_CRITICAL;
    } else if (report.min_separation < cfg->safe_separation_m) {
        report.conflict = 1u;
        report.risk = SWARM_RISK_WARNING;
    }
    return report;
}
