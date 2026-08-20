/* Host-only deterministic scenario and module configuration. */
#ifndef SCENARIO_H
#define SCENARIO_H

#include "nav_math.h"
#include "waypoint.h"
#include "mission_fsm.h"
#include "state_estimator.h"
#include "impact_detector.h"
#include "impact_recovery.h"
#include "safety_monitor.h"
#include "obstacle_avoidance.h"
#include "trajectory_planner.h"
#include "pos_controller.h"
#include "guidance.h"
#include "camera.h"
#include "vision_frontend.h"
#include "sim_dynamics.h"
#include "collision_interface.h"
#include "swarm_avoidance.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SCENARIO_NOMINAL = 0,
    SCENARIO_MOVING_TARGET,
    SCENARIO_TARGET_LOSS,
    SCENARIO_IMPACT_DEGRADED,
    SCENARIO_IMPACT_LOST,
    SCENARIO_HOME_INITIAL_HIDDEN,
    SCENARIO_HOME_LOSS,
    SCENARIO_LOCAL_OBSTACLE,
    SCENARIO_FORCED_RETURN,
    SCENARIO_TWO_AGENT_CONFLICT
} ScenarioKind;

typedef struct {
    ScenarioKind kind;
    const char *name;
    Vec3f home_pos;
    Vec3f target_pos;
    Vec3f target_velocity;
    WaypointQueue outbound_route;
    WaypointQueue search_route;

    MissionConfig mission;
    EstimatorConfig estimator;
    ImpactDetectorConfig impact;
    ImpactFusionConfig impact_fusion;
    ImpactRecoveryConfig recovery;
    SafetyConfig safety;
    PosCtrlParams ctrl;
    SimParams dynamics;
    TerminalParams terminal;
    HomeParams home_guidance;
    TrajectoryPlannerConfig planner;
    ObstacleAvoidanceConfig obstacle_avoidance;
    DynamicObstacleSet obstacles;
    CollisionConfig swarm_collision;
    SwarmAvoidanceConfig swarm_avoidance;

    CameraModel cam_forward;
    CameraModel cam_down;
    float target_size_m;
    float marker_size_m;
    uint8_t use_flow_vo;
    FlowConfig flow;
    float feature_area_m;
    uint16_t feature_count;

    float dt;
    float sim_max_time_s;
    float impact_range_m;
    float vision_freeze_s;
    uint32_t seed;
    Vec3f impact_delta_v;
    float impact_delta_yaw;
    float impact_delta_pitch;
    float impact_delta_roll;

    float target_hidden_start_s;
    float target_hidden_end_s;
    float home_hidden_until_s;
    float home_loss_start_s;
    float home_loss_end_s;
    uint8_t other_agent_enabled;
    AgentState other_agent;
} Scenario;

void scenario_default(Scenario *scenario);
int scenario_apply_kind(Scenario *scenario, const char *name);
const char *scenario_kind_name(ScenarioKind kind);

#ifdef __cplusplus
}
#endif

#endif /* SCENARIO_H */
