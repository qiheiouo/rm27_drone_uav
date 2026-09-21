/* Platform-neutral, fixed-memory navigation runtime. */
#ifndef NAV_RUNTIME_H
#define NAV_RUNTIME_H

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
#include "swarm_link_guard.h"
#include "nav_event_log.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    NAV_CONFIG_ERROR_NONE = 0u,
    NAV_CONFIG_ERROR_ARGUMENT = 1u << 0,
    NAV_CONFIG_ERROR_TIMING = 1u << 1,
    NAV_CONFIG_ERROR_MISSION = 1u << 2,
    NAV_CONFIG_ERROR_ESTIMATOR = 1u << 3,
    NAV_CONFIG_ERROR_SAFETY = 1u << 4,
    NAV_CONFIG_ERROR_CONTROL = 1u << 5,
    NAV_CONFIG_ERROR_GUIDANCE = 1u << 6,
    NAV_CONFIG_ERROR_PLANNER = 1u << 7,
    NAV_CONFIG_ERROR_OBSTACLE = 1u << 8,
    NAV_CONFIG_ERROR_CAMERA = 1u << 9,
    NAV_CONFIG_ERROR_ROUTE = 1u << 10,
    NAV_CONFIG_ERROR_SWARM = 1u << 11
};

enum {
    NAV_EVENT_NONE = 0u,
    NAV_EVENT_IMPACT_CONFIRMED = 1u << 0,
    NAV_EVENT_RELOCALIZED = 1u << 1,
    NAV_EVENT_OBSTACLE_RISK = 1u << 2,
    NAV_EVENT_SWARM_CONFLICT = 1u << 3,
    NAV_EVENT_TRAJECTORY_DETOUR = 1u << 4,
    NAV_EVENT_TRAJECTORY_INVALID = 1u << 5,
    NAV_EVENT_INPUT_REJECTED = 1u << 6,
    NAV_EVENT_WATCHDOG_OVERRUN = 1u << 7,
    NAV_EVENT_WATCHDOG_TRIPPED = 1u << 8,
    NAV_EVENT_SWARM_LINK_GUARD = 1u << 9
};

enum {
    NAV_INPUT_SOURCE_IMU = 1u << 0,
    NAV_INPUT_SOURCE_FLOW = 1u << 1,
    NAV_INPUT_SOURCE_ODOMETRY = 1u << 2,
    NAV_INPUT_SOURCE_TARGET = 1u << 3,
    NAV_INPUT_SOURCE_HOME = 1u << 4
};

#define NAV_TIMESTAMPED_INPUT_COUNT 5u

typedef struct {
    uint32_t imu_max_age_ms;
    uint32_t flow_max_age_ms;
    uint32_t odometry_max_age_ms;
    uint32_t vision_max_age_ms;
    uint32_t future_tolerance_ms;
    uint32_t max_cycle_gap_ms;
    uint8_t watchdog_trip_after_overruns;
} NavRuntimeHealthConfig;

typedef struct {
    uint32_t stale_source_mask;
    uint32_t duplicate_source_mask;
    uint32_t out_of_order_source_mask;
    uint32_t nonfinite_source_mask;
    uint32_t invalid_source_mask;
    uint32_t cycle_gap_ms;
    uint8_t consecutive_overruns;
    uint8_t watchdog_tripped;
    uint8_t required_input_valid;
} NavRuntimeHealth;

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
    SwarmLinkGuardConfig swarm_link_guard;
    PosCtrlParams ctrl;
    TerminalParams terminal;
    HomeParams home_guidance;
    FlowConfig flow;
    CameraModel cam_forward;
    CameraModel cam_down;
    float target_size_m;
    float marker_size_m;
    float target_filter_cutoff_hz;
    float home_filter_cutoff_hz;
    float nominal_dt_s;
    float home_yaw_rad;
    float relocalization_cooldown_s;
    float relocalization_min_correction_m;
    NavRuntimeHealthConfig health;
    Vec3f home_pos;
    WaypointQueue outbound_route;
    WaypointQueue search_route;
    uint8_t self_agent_id;
} NavRuntimeConfig;

/* A null optional sample means that source is unavailable for this update. */
typedef struct {
    uint32_t timestamp_ms;
    const ImuSample *imu;
    const FlowFrame *flow;
    const OdomSample *odometry;
    float tof_height;
    const PixelObs *target_pixel;
    const PixelObs *home_pixel;
    const DynamicObstacleSet *obstacles;
    const SwarmView *swarm;
    uint8_t start_command;
    uint8_t request_return;
    uint8_t request_emergency;
    uint8_t dock_contact;
    uint8_t charging_detected;
    uint8_t wireless_charge_ready;
    float dt;
} NavRuntimeInput;

typedef struct {
    NavState nav;
    MissionOutput mission;
    SafetyDecision safety;
    ImpactReport impact;
    ImpactRecoveryOutput recovery;
    TerminalStage terminal_stage;
    TargetTrack target;
    HomeTrack home;
    ObstacleRiskReport obstacle;
    CollisionReport swarm_collision;
    SwarmAvoidanceDecision swarm_avoidance;
    SwarmLinkGuardOutput swarm_link_guard;
    TrajectoryPlanReport trajectory;
    GuidanceOutput guidance;
    CtrlOutput control;
    AgentState swarm_self;
    NavRuntimeHealth health;
    uint32_t event_flags;
    uint8_t recovery_active;
    uint8_t trajectory_active;
    uint8_t trajectory_valid;
    uint8_t armed;
    uint8_t step_valid;
} NavRuntimeOutput;

typedef struct {
    NavRuntimeConfig cfg;
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
    SwarmLinkGuard swarm_link_guard;
    Trajectory trajectory;
    TrajectoryPlanReport trajectory_report;
    CtrlOutput previous_control;
    MissionOutput mission_output;
    NavRuntimeOutput output;
    NavEventLog event_log;
    float trajectory_time_s;
    uint32_t last_relocalization_ms;
    uint32_t last_update_timestamp_ms;
    uint32_t last_source_timestamp_ms[NAV_TIMESTAMPED_INPUT_COUNT];
    uint32_t source_seen_mask;
    uint32_t config_errors;
    uint8_t update_timestamp_seen;
    uint8_t watchdog_overruns;
    uint8_t watchdog_tripped;
    uint8_t trajectory_active;
    uint8_t trajectory_valid;
    uint8_t relocalization_seen;
    uint8_t initialized;
} NavRuntime;

void nav_runtime_config_default(NavRuntimeConfig *cfg);
uint32_t nav_runtime_config_validate(const NavRuntimeConfig *cfg);
const char *nav_runtime_config_error_name(uint32_t single_error);

/* Returns the validation error mask; zero means initialization succeeded. */
uint32_t nav_runtime_init(NavRuntime *runtime, const NavRuntimeConfig *cfg);

/* Returns 1 after a valid update, or 0 when initialization/input is invalid. */
uint8_t nav_runtime_step(NavRuntime *runtime, const NavRuntimeInput *input);
const NavEventLog *nav_runtime_event_log(const NavRuntime *runtime);

#ifdef __cplusplus
}
#endif

#endif /* NAV_RUNTIME_H */
