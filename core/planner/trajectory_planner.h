/* Bounded trajectory generation, post-check and conservative fallback. */
#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include <stdint.h>
#include "trajectory.h"
#include "obstacle_avoidance.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TRAJ_BACKEND_SIMPLE = 0,
    TRAJ_BACKEND_OPTIMIZED
} TrajectoryBackend;

enum {
    TRAJ_CHECK_OK = 0u,
    TRAJ_CHECK_NONFINITE = 1u << 0,
    TRAJ_CHECK_DURATION = 1u << 1,
    TRAJ_CHECK_VELOCITY = 1u << 2,
    TRAJ_CHECK_ACCELERATION = 1u << 3,
    TRAJ_CHECK_WORKSPACE = 1u << 4,
    TRAJ_CHECK_COLLISION = 1u << 5
};

typedef struct {
    float max_duration_s;
    float max_velocity_mps;
    float max_acceleration_mps2;
    float sample_dt_s;
    Vec3f workspace_min;
    Vec3f workspace_max;
    float collision_clearance_m;
} TrajectoryLimits;

typedef struct {
    uint8_t flags;
    float peak_velocity_mps;
    float peak_acceleration_mps2;
    float minimum_clearance_m;
    float checked_duration_s;
} TrajectoryCheckReport;

typedef struct {
    TrajectoryBackend requested_backend;
    TrajectoryBackend used_backend;
    uint8_t fallback_used;
    uint8_t detour_used;
    uint8_t iterations;
    uint8_t valid;
    TrajectoryCheckReport check;
} TrajectoryPlanReport;

typedef struct {
    TrajectoryLimits limits;
    float optimized_initial_time_scale;
    float simple_time_scale;
    float time_stretch_factor;
    float detour_margin_m;
    uint8_t max_iterations;
} TrajectoryPlannerConfig;

void trajectory_planner_default_config(TrajectoryPlannerConfig *cfg);
TrajectoryCheckReport trajectory_postcheck(const Trajectory *trajectory,
                                            const TrajectoryLimits *limits,
                                            const DynamicObstacleSet *obstacles);
TrajectoryPlanReport trajectory_plan_route(Trajectory *trajectory,
                                           Vec3f start_position,
                                           Vec3f start_velocity,
                                           const WaypointQueue *route,
                                           const TrajectoryPlannerConfig *cfg,
                                           const DynamicObstacleSet *obstacles,
                                           TrajectoryBackend backend);
TrajectoryPlanReport trajectory_plan_single(Trajectory *trajectory,
                                            Vec3f start_position,
                                            Vec3f start_velocity,
                                            Vec3f goal_position,
                                            float speed_mps,
                                            const TrajectoryPlannerConfig *cfg,
                                            const DynamicObstacleSet *obstacles,
                                            TrajectoryBackend backend);

#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_PLANNER_H */
