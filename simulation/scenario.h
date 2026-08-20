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
#include "nav_runtime.h"
#include "fault_injection.h"

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
    SCENARIO_TWO_AGENT_CONFLICT,
    SCENARIO_FLOW_DROPOUT,
    SCENARIO_TARGET_FREEZE,
    SCENARIO_HOME_DELAY,
    SCENARIO_NAN_TARGET,
    SCENARIO_IMU_STALE,
    SCENARIO_IMU_DUPLICATE,
    SCENARIO_IMU_ROLLBACK,
    SCENARIO_WATCHDOG_OVERRUN
} ScenarioKind;

typedef enum {
    SCENARIO_EXPECT_COMPLETE = 0,
    SCENARIO_EXPECT_RUNTIME_REJECT,
    SCENARIO_EXPECT_EMERGENCY_LAND
} ScenarioExpectedOutcome;

typedef struct {
    ScenarioExpectedOutcome outcome;
    uint32_t required_fault_rule_mask;
    uint32_t required_event_flags;
    uint32_t required_log_code_mask;
    uint32_t required_stale_source_mask;
    uint32_t required_duplicate_source_mask;
    uint32_t required_out_of_order_source_mask;
    uint32_t required_nonfinite_source_mask;
    uint32_t required_invalid_source_mask;
    uint8_t require_estimator_degraded;
    uint8_t require_estimator_recovered;
    uint8_t require_target_unavailable;
    uint8_t require_target_recovered;
    uint8_t require_home_unavailable;
    uint8_t require_home_recovered;
} ScenarioExpectations;

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
    NavRuntimeHealthConfig health;
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
    SimFaultPlan fault_plan;
    ScenarioExpectations expectations;
} Scenario;

void scenario_default(Scenario *scenario);
void scenario_runtime_config(const Scenario *scenario, NavRuntimeConfig *cfg);
int scenario_apply_kind(Scenario *scenario, const char *name);
const char *scenario_kind_name(ScenarioKind kind);

#ifdef __cplusplus
}
#endif

#endif /* SCENARIO_H */
