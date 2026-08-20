/* Fixed-capacity local/dynamic obstacle abstraction and reactive avoidance. */
#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <stdint.h>
#include "nav_math.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LOCAL_OBSTACLE_MAX 8u

typedef struct {
    uint8_t valid;
    uint8_t obstacle_id;
    Vec3f relative_position;
    Vec3f relative_velocity;
    float radius_m;
    float confidence;
    float age_s;
} LocalObstacle;

typedef struct {
    uint8_t valid;
    uint8_t obstacle_id;
    Vec3f position;
    Vec3f velocity;
    float radius_m;
    float confidence;
    float age_s;
} DynamicObstacle;

typedef struct {
    DynamicObstacle items[LOCAL_OBSTACLE_MAX];
    uint8_t count;
} DynamicObstacleSet;

typedef enum {
    COLLISION_RISK_NONE = 0,
    COLLISION_RISK_WARNING,
    COLLISION_RISK_CRITICAL
} CollisionRiskLevel;

typedef struct {
    CollisionRiskLevel level;
    uint8_t obstacle_id;
    float minimum_separation_m;
    float time_to_closest_s;
    Vec3f avoidance_velocity;
} ObstacleRiskReport;

typedef struct {
    float vehicle_radius_m;
    float warning_clearance_m;
    float emergency_clearance_m;
    float prediction_horizon_s;
    float max_avoidance_speed_mps;
    float emergency_climb_speed_mps;
    float min_confidence;
    float max_observation_age_s;
} ObstacleAvoidanceConfig;

void obstacle_set_init(DynamicObstacleSet *set);
int obstacle_set_push(DynamicObstacleSet *set, const DynamicObstacle *obstacle);
void obstacle_avoidance_default_config(ObstacleAvoidanceConfig *cfg);
ObstacleRiskReport obstacle_evaluate(const ObstacleAvoidanceConfig *cfg,
                                     Vec3f self_position,
                                     Vec3f self_velocity,
                                     Vec3f commanded_velocity,
                                     const DynamicObstacleSet *obstacles);
Vec3f obstacle_apply_avoidance(Vec3f commanded_velocity,
                               const ObstacleRiskReport *report,
                               float max_speed_mps);

#ifdef __cplusplus
}
#endif

#endif /* OBSTACLE_AVOIDANCE_H */
