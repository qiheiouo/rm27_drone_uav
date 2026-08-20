#include "trajectory_planner.h"

void trajectory_planner_default_config(TrajectoryPlannerConfig *cfg)
{
    cfg->limits.max_duration_s = 20.0f;
    cfg->limits.max_velocity_mps = 2.2f;
    cfg->limits.max_acceleration_mps2 = 6.0f;
    cfg->limits.sample_dt_s = 0.05f;
    cfg->limits.workspace_min = vec3(-12.0f, -12.0f, 0.0f);
    cfg->limits.workspace_max = vec3(12.0f, 12.0f, 3.0f);
    cfg->limits.collision_clearance_m = 0.20f;
    cfg->optimized_initial_time_scale = 1.25f;
    cfg->simple_time_scale = 2.2f;
    cfg->time_stretch_factor = 1.35f;
    cfg->detour_margin_m = 0.45f;
    cfg->max_iterations = 4u;
}

static float point_segment_distance_xy(Vec3f point, Vec3f start, Vec3f end)
{
    Vec3f segment = vec3_sub(end, start);
    Vec3f relative = vec3_sub(point, start);
    float length_sq = segment.x * segment.x + segment.y * segment.y;
    float fraction = length_sq > 1e-6f
        ? clampf((relative.x * segment.x + relative.y * segment.y) / length_sq,
                 0.0f, 1.0f)
        : 0.0f;
    Vec3f closest = vec3(start.x + fraction * segment.x,
                         start.y + fraction * segment.y,
                         point.z);
    return vec3_dist_xy(point, closest);
}

/* Build a bounded, deterministic lateral detour around locally known obstacles.
 * This is deliberately small enough for an MCU: no heap and no graph search. */
static uint8_t build_detour_route(WaypointQueue *detour,
                                  Vec3f start_position,
                                  const WaypointQueue *route,
                                  const TrajectoryPlannerConfig *cfg,
                                  const DynamicObstacleSet *obstacles)
{
    Vec3f segment_start = start_position;
    uint8_t used = 0u;
    uint8_t route_index;

    wq_init(detour);
    for (route_index = route->index; route_index < route->count; route_index++) {
        const Waypoint *goal = &route->items[route_index];
        uint8_t obstacle_index;
        for (obstacle_index = 0u;
             obstacles != 0 && obstacle_index < obstacles->count;
             obstacle_index++) {
            const DynamicObstacle *obstacle = &obstacles->items[obstacle_index];
            Vec3f direction;
            Vec3f lateral;
            Vec3f bypass;
            float trigger_distance;
            float bypass_distance;
            if (!obstacle->valid || obstacle->confidence < 0.3f ||
                obstacle->age_s > 0.5f) {
                continue;
            }
            trigger_distance = obstacle->radius_m +
                               cfg->limits.collision_clearance_m;
            if (point_segment_distance_xy(obstacle->position, segment_start,
                                          goal->pos) >= trigger_distance) {
                continue;
            }
            direction = vec3_normalize_or(
                vec3(goal->pos.x - segment_start.x,
                     goal->pos.y - segment_start.y, 0.0f),
                vec3(1.0f, 0.0f, 0.0f));
            lateral = vec3(-direction.y, direction.x, 0.0f);
            if ((obstacle->obstacle_id & 1u) == 0u) {
                lateral = vec3_scale(lateral, -1.0f);
            }
            bypass_distance = obstacle->radius_m +
                              cfg->limits.collision_clearance_m +
                              cfg->detour_margin_m;
            bypass = vec3_add(obstacle->position,
                              vec3_scale(lateral, bypass_distance));
            bypass.z = 0.5f * (segment_start.z + goal->pos.z);
            bypass.x = clampf(bypass.x, cfg->limits.workspace_min.x,
                              cfg->limits.workspace_max.x);
            bypass.y = clampf(bypass.y, cfg->limits.workspace_min.y,
                              cfg->limits.workspace_max.y);
            bypass.z = clampf(bypass.z, cfg->limits.workspace_min.z,
                              cfg->limits.workspace_max.z);
            if (wq_push(detour, bypass, goal->speed) != 0) {
                return 0u;
            }
            segment_start = bypass;
            used = 1u;
        }
        if (wq_push(detour, goal->pos, goal->speed) != 0) {
            return 0u;
        }
        segment_start = goal->pos;
    }
    return used;
}

static uint8_t outside_workspace(Vec3f position, const TrajectoryLimits *limits)
{
    return (position.x < limits->workspace_min.x || position.x > limits->workspace_max.x ||
            position.y < limits->workspace_min.y || position.y > limits->workspace_max.y ||
            position.z < limits->workspace_min.z || position.z > limits->workspace_max.z) ? 1u : 0u;
}

TrajectoryCheckReport trajectory_postcheck(const Trajectory *trajectory,
                                            const TrajectoryLimits *limits,
                                            const DynamicObstacleSet *obstacles)
{
    TrajectoryCheckReport report;
    float sample_dt = (limits->sample_dt_s >= 0.01f) ? limits->sample_dt_s : 0.05f;
    float time;

    report.flags = TRAJ_CHECK_OK;
    report.peak_velocity_mps = 0.0f;
    report.peak_acceleration_mps2 = 0.0f;
    report.minimum_clearance_m = 1e9f;
    report.checked_duration_s = trajectory->duration;

    if (trajectory->count == 0u || !nav_isfinite(trajectory->duration) ||
        trajectory->duration <= 0.0f || trajectory->duration > limits->max_duration_s) {
        report.flags |= TRAJ_CHECK_DURATION;
    }

    for (time = 0.0f; time <= trajectory->duration + 0.5f * sample_dt; time += sample_dt) {
        Vec3f position;
        Vec3f velocity;
        Vec3f acceleration;
        uint8_t index;
        traj_evaluate(trajectory, time, &position, &velocity, &acceleration);
        if (!vec3_is_finite(position) || !vec3_is_finite(velocity) ||
            !vec3_is_finite(acceleration)) {
            report.flags |= TRAJ_CHECK_NONFINITE;
            continue;
        }
        if (vec3_norm(velocity) > report.peak_velocity_mps) {
            report.peak_velocity_mps = vec3_norm(velocity);
        }
        if (vec3_norm(acceleration) > report.peak_acceleration_mps2) {
            report.peak_acceleration_mps2 = vec3_norm(acceleration);
        }
        if (outside_workspace(position, limits)) {
            report.flags |= TRAJ_CHECK_WORKSPACE;
        }
        if (obstacles != 0) {
            for (index = 0u; index < obstacles->count && index < LOCAL_OBSTACLE_MAX; index++) {
                const DynamicObstacle *obstacle = &obstacles->items[index];
                Vec3f obstacle_position;
                float clearance;
                if (!obstacle->valid) {
                    continue;
                }
                obstacle_position = vec3_add(obstacle->position,
                                             vec3_scale(obstacle->velocity, time));
                clearance = vec3_dist(position, obstacle_position) - obstacle->radius_m;
                if (clearance < report.minimum_clearance_m) {
                    report.minimum_clearance_m = clearance;
                }
                if (clearance < limits->collision_clearance_m) {
                    report.flags |= TRAJ_CHECK_COLLISION;
                }
            }
        }
    }

    if (report.peak_velocity_mps > limits->max_velocity_mps) {
        report.flags |= TRAJ_CHECK_VELOCITY;
    }
    if (report.peak_acceleration_mps2 > limits->max_acceleration_mps2) {
        report.flags |= TRAJ_CHECK_ACCELERATION;
    }
    return report;
}

static TrajectoryPlanReport route_plan(Trajectory *trajectory,
                                       Vec3f start_position,
                                       Vec3f start_velocity,
                                       const WaypointQueue *route,
                                       const TrajectoryPlannerConfig *cfg,
                                       const DynamicObstacleSet *obstacles,
                                       TrajectoryBackend backend)
{
    TrajectoryPlanReport report;
    WaypointQueue detour_route;
    const WaypointQueue *working_route = route;
    float time_scale = (backend == TRAJ_BACKEND_OPTIMIZED)
        ? cfg->optimized_initial_time_scale : cfg->simple_time_scale;
    uint8_t iteration;

    report.requested_backend = backend;
    report.used_backend = backend;
    report.fallback_used = 0u;
    report.detour_used = 0u;
    report.iterations = 0u;
    report.valid = 0u;

    if (backend == TRAJ_BACKEND_OPTIMIZED && obstacles != 0 &&
        obstacles->count > 0u &&
        build_detour_route(&detour_route, start_position, route, cfg, obstacles)) {
        working_route = &detour_route;
        report.detour_used = 1u;
    }

    for (iteration = 0u; iteration < cfg->max_iterations; iteration++) {
        traj_build(trajectory, start_position, start_velocity, working_route, time_scale);
        report.check = trajectory_postcheck(trajectory, &cfg->limits, obstacles);
        report.iterations = (uint8_t)(iteration + 1u);
        if (report.check.flags == TRAJ_CHECK_OK) {
            report.valid = 1u;
            return report;
        }
        if ((report.check.flags & ~(TRAJ_CHECK_VELOCITY | TRAJ_CHECK_ACCELERATION)) != 0u) {
            break;
        }
        time_scale *= cfg->time_stretch_factor;
    }

    if (backend == TRAJ_BACKEND_OPTIMIZED) {
        traj_build(trajectory, start_position, start_velocity, working_route,
                   cfg->simple_time_scale);
        report.check = trajectory_postcheck(trajectory, &cfg->limits, obstacles);
        report.used_backend = TRAJ_BACKEND_SIMPLE;
        report.fallback_used = 1u;
        report.valid = (report.check.flags == TRAJ_CHECK_OK) ? 1u : 0u;
    }
    return report;
}

TrajectoryPlanReport trajectory_plan_route(Trajectory *trajectory,
                                           Vec3f start_position,
                                           Vec3f start_velocity,
                                           const WaypointQueue *route,
                                           const TrajectoryPlannerConfig *cfg,
                                           const DynamicObstacleSet *obstacles,
                                           TrajectoryBackend backend)
{
    return route_plan(trajectory, start_position, start_velocity, route,
                      cfg, obstacles, backend);
}

TrajectoryPlanReport trajectory_plan_single(Trajectory *trajectory,
                                            Vec3f start_position,
                                            Vec3f start_velocity,
                                            Vec3f goal_position,
                                            float speed_mps,
                                            const TrajectoryPlannerConfig *cfg,
                                            const DynamicObstacleSet *obstacles,
                                            TrajectoryBackend backend)
{
    WaypointQueue route;
    wq_init(&route);
    wq_push(&route, goal_position, speed_mps);
    return route_plan(trajectory, start_position, start_velocity, &route,
                      cfg, obstacles, backend);
}
