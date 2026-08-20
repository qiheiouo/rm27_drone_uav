/* Fixed-memory STM32 navigation task adapter. */
#ifndef NAV_TASKS_H
#define NAV_TASKS_H

#include <stdint.h>
#include "state_estimator.h"
#include "impact_detector.h"
#include "impact_recovery.h"
#include "safety_monitor.h"
#include "obstacle_avoidance.h"
#include "target_tracker.h"
#include "home_detector.h"
#include "vision_frontend.h"
#include "mission_fsm.h"
#include "pos_controller.h"
#include "trajectory_planner.h"
#include "camera.h"
#include "guidance.h"
#include "collision_interface.h"
#include "swarm_avoidance.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    MissionConfig mission;
    EstimatorConfig estimator;
    ImpactDetectorConfig impact;
    ImpactFusionConfig impact_fusion;
    ImpactRecoveryConfig recovery;
    SafetyConfig safety;
    ObstacleAvoidanceConfig obstacle_avoidance;
    TrajectoryPlannerConfig planner;
    CollisionConfig swarm_collision;
    SwarmAvoidanceConfig swarm_avoidance;
    PosCtrlParams ctrl;
    TerminalParams terminal;
    HomeParams home_guidance;
    FlowConfig flow;
    CameraModel cam_forward;
    CameraModel cam_down;
    float target_size_m;
    float marker_size_m;
    Vec3f home_pos;
    WaypointQueue outbound_route;
    WaypointQueue search_route;
    uint8_t self_agent_id;
} NavAppConfig;

typedef struct {
    NavAppConfig cfg;
    StateEstimator estimator;
    ImpactDetector impact_detector;
    ImpactRecovery recovery;
    ImpactRecoveryOutput recovery_output;
    TargetTracker target_tracker;
    HomeDetector home_detector;
    SafetyMonitor safety;
    MissionFsm mission;
    TerminalGuidance terminal;
    VisionFrontend vision_frontend;
    DynamicObstacleSet obstacles;
    SwarmView swarm;
    Trajectory trajectory;
    TrajectoryPlanReport trajectory_report;
    CtrlOutput previous_control;
    MissionOutput mission_output;
    float trajectory_time_s;
    uint32_t last_relocalization_ms;
    uint8_t trajectory_active;
    uint8_t trajectory_valid;
} NavApp;

void nav_app_init(NavApp *app, const NavAppConfig *cfg);

/* Call at 100--200 Hz. All expensive loops have compile-time bounds. */
void nav_app_step(NavApp *app, float dt);

#ifdef __cplusplus
}
#endif

#endif /* NAV_TASKS_H */
