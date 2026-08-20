#include "obstacle_avoidance.h"

void obstacle_set_init(DynamicObstacleSet *set)
{
    set->count = 0u;
}

int obstacle_set_push(DynamicObstacleSet *set, const DynamicObstacle *obstacle)
{
    if (set->count >= LOCAL_OBSTACLE_MAX) {
        return -1;
    }
    set->items[set->count++] = *obstacle;
    return 0;
}

void obstacle_avoidance_default_config(ObstacleAvoidanceConfig *cfg)
{
    cfg->vehicle_radius_m = 0.10f;
    cfg->warning_clearance_m = 0.55f;
    cfg->emergency_clearance_m = 0.20f;
    cfg->prediction_horizon_s = 2.0f;
    cfg->max_avoidance_speed_mps = 0.8f;
    cfg->emergency_climb_speed_mps = 0.6f;
    cfg->min_confidence = 0.3f;
    cfg->max_observation_age_s = 0.5f;
}

ObstacleRiskReport obstacle_evaluate(const ObstacleAvoidanceConfig *cfg,
                                     Vec3f self_position,
                                     Vec3f self_velocity,
                                     Vec3f commanded_velocity,
                                     const DynamicObstacleSet *obstacles)
{
    ObstacleRiskReport report;
    uint8_t index;
    Vec3f own_velocity = vec3_lerp(self_velocity, commanded_velocity, 0.7f);

    report.level = COLLISION_RISK_NONE;
    report.obstacle_id = 0u;
    report.minimum_separation_m = 1e9f;
    report.time_to_closest_s = cfg->prediction_horizon_s;
    report.avoidance_velocity = vec3_zero();

    for (index = 0u; index < obstacles->count && index < LOCAL_OBSTACLE_MAX; index++) {
        const DynamicObstacle *obstacle = &obstacles->items[index];
        Vec3f relative_position;
        Vec3f relative_velocity;
        float relative_speed_sq;
        float closest_time;
        Vec3f closest_vector;
        float separation;

        if (!obstacle->valid || obstacle->confidence < cfg->min_confidence ||
            obstacle->age_s > cfg->max_observation_age_s) {
            continue;
        }

        relative_position = vec3_sub(obstacle->position, self_position);
        relative_velocity = vec3_sub(obstacle->velocity, own_velocity);
        relative_speed_sq = vec3_dot(relative_velocity, relative_velocity);
        closest_time = relative_speed_sq > 1e-6f
            ? clampf(-vec3_dot(relative_position, relative_velocity) / relative_speed_sq,
                     0.0f, cfg->prediction_horizon_s)
            : 0.0f;
        closest_vector = vec3_add(relative_position,
                                  vec3_scale(relative_velocity, closest_time));
        separation = vec3_norm(closest_vector) - obstacle->radius_m - cfg->vehicle_radius_m;

        if (separation < report.minimum_separation_m) {
            Vec3f away = vec3_normalize_or(vec3_scale(closest_vector, -1.0f),
                                           vec3(0.0f, 1.0f, 0.0f));
            Vec3f lateral = vec3_normalize_or(vec3(-relative_position.y,
                                                    relative_position.x, 0.0f), away);
            float strength = clampf((cfg->warning_clearance_m - separation) /
                                    (cfg->warning_clearance_m + 1e-3f), 0.0f, 1.0f);
            report.minimum_separation_m = separation;
            report.time_to_closest_s = closest_time;
            report.obstacle_id = obstacle->obstacle_id;
            report.avoidance_velocity = vec3_scale(lateral,
                cfg->max_avoidance_speed_mps * strength);
            if (separation <= cfg->emergency_clearance_m) {
                report.level = COLLISION_RISK_CRITICAL;
                report.avoidance_velocity = vec3_add(
                    vec3_scale(away, cfg->max_avoidance_speed_mps),
                    vec3(0.0f, 0.0f, cfg->emergency_climb_speed_mps));
            } else if (separation <= cfg->warning_clearance_m) {
                report.level = COLLISION_RISK_WARNING;
            } else {
                report.level = COLLISION_RISK_NONE;
            }
        }
    }
    return report;
}

Vec3f obstacle_apply_avoidance(Vec3f commanded_velocity,
                               const ObstacleRiskReport *report,
                               float max_speed_mps)
{
    if (report->level == COLLISION_RISK_NONE) {
        return commanded_velocity;
    }
    if (report->level == COLLISION_RISK_CRITICAL) {
        return vec3_clamp_norm(report->avoidance_velocity, max_speed_mps);
    }
    return vec3_clamp_norm(vec3_add(commanded_velocity,
                                    report->avoidance_velocity), max_speed_mps);
}
