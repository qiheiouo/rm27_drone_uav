#include <stdio.h>
#include "scenario.h"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; \
} } while (0)

int main(void)
{
    TrajectoryPlannerConfig config;
    DynamicObstacleSet obstacles;
    DynamicObstacle obstacle = {0};
    Trajectory trajectory;
    TrajectoryPlanReport plan;
    ObstacleAvoidanceConfig avoidance;
    ObstacleRiskReport risk;
    Vec3f command;

    trajectory_planner_default_config(&config);
    obstacle_set_init(&obstacles);
    obstacle.valid = 1u;
    obstacle.obstacle_id = 3u;
    obstacle.position = vec3(2.0f, 0.0f, 1.0f);
    obstacle.radius_m = 0.25f;
    obstacle.confidence = 1.0f;
    CHECK(obstacle_set_push(&obstacles, &obstacle) == 0);

    plan = trajectory_plan_single(&trajectory, vec3(0.0f, 0.0f, 1.0f),
        vec3_zero(), vec3(4.0f, 0.0f, 1.0f), 1.5f,
        &config, &obstacles, TRAJ_BACKEND_OPTIMIZED);
    CHECK(plan.detour_used == 1u);
    CHECK(plan.valid == 1u);
    CHECK(plan.check.flags == TRAJ_CHECK_OK);
    CHECK(trajectory.count == 2u);

    obstacle_avoidance_default_config(&avoidance);
    risk = obstacle_evaluate(&avoidance, vec3(1.0f, 0.0f, 1.0f),
                             vec3(1.0f, 0.0f, 0.0f),
                             vec3(1.0f, 0.0f, 0.0f), &obstacles);
    CHECK(risk.level != COLLISION_RISK_NONE);
    command = obstacle_apply_avoidance(vec3(1.0f, 0.0f, 0.0f),
                                       &risk, 2.0f);
    CHECK(fabsf(command.y) > 0.01f || command.z > 0.01f);
    CHECK(vec3_norm(command) <= 2.0f + 1e-4f);

    if (failures == 0) {
        printf("test_planner_avoidance: PASS\n");
        return 0;
    }
    printf("test_planner_avoidance: %d FAILURES\n", failures);
    return 1;
}
