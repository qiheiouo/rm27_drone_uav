/* Bounded hierarchical FSM for the complete Plan-B sortie. */
#ifndef MISSION_FSM_H
#define MISSION_FSM_H

#include <stdint.h>
#include "nav_math.h"
#include "state_estimator.h"
#include "target_tracker.h"
#include "home_detector.h"
#include "waypoint.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MS_BOOT = 0,
    MS_SELF_CHECK,
    MS_DOCKED,
    MS_TAKEOFF,
    MS_OUTBOUND,
    MS_SEARCH,
    MS_TARGET_ACQUIRE,
    MS_TARGET_TRACK,
    MS_TERMINAL,
    MS_IMPACT,
    MS_RECOVERY,
    MS_BREAKAWAY,
    MS_RETURN_HOME,
    MS_HOME_SEARCH,
    MS_HOMING,
    MS_DOCKING,
    MS_ESTIMATOR_LOST,
    MS_EMERGENCY_STABILIZE,
    MS_EMERGENCY_LAND
} MissionState;

typedef enum {
    GM_NONE = 0,
    GM_HOLD,
    GM_TAKEOFF,
    GM_WAYPOINT,
    GM_TERMINAL,
    GM_RECOVERY,
    GM_HOME_SERVO,
    GM_LAND
} GuidanceMode;

typedef enum {
    RETURN_TRANSIT = 0,
    RETURN_HOME_APPROACH,
    RETURN_HOME_SEARCH,
    RETURN_HOME_ACQUIRED,
    RETURN_HOMING
} ReturnStage;

typedef enum {
    HOMING_COARSE = 0,
    HOMING_FINE_ALIGNMENT,
    HOMING_DESCENT,
    HOMING_FINAL_APPROACH
} HomingStage;

typedef enum {
    DOCK_SEARCH_MARKER = 0,
    DOCK_APPROACH,
    DOCK_ALIGN,
    DOCK_DESCEND,
    DOCK_FINAL_CAPTURE,
    DOCK_CONTACT,
    DOCK_DOCKED
} DockingStage;

typedef enum {
    MISSION_REASON_NONE = 0,
    MISSION_REASON_NOMINAL,
    MISSION_REASON_TARGET_FOUND,
    MISSION_REASON_TARGET_LOST,
    MISSION_REASON_IMPACT,
    MISSION_REASON_RECOVERY_COMPLETE,
    MISSION_REASON_HOME_ACQUIRED,
    MISSION_REASON_HOME_LOST,
    MISSION_REASON_SOFT_RETURN,
    MISSION_REASON_ESTIMATOR_LOST,
    MISSION_REASON_NAV_FAILURE,
    MISSION_REASON_EMERGENCY
} MissionTransitionReason;

typedef struct {
    float takeoff_alt_m;
    float waypoint_tol_m;
    float cruise_speed_mps;
    float search_yaw_rate_rps;
    float target_confirm_s;
    float target_acquire_timeout_s;
    float terminal_range_m;
    float target_lost_timeout_s;
    float recovery_hold_s;
    float recovery_tilt_ok_rad;
    float breakaway_height_m;
    float home_region_tol_m;
    float home_approach_radius_m;
    float home_search_alt_m;
    float homing_fine_radius_m;
    float homing_yaw_tol_rad;
    float dock_alt_m;
    float dock_lateral_tol_m;
    float dock_capture_tol_m;
    float dock_capture_max_alt_m;
    float dock_land_vel_max;
    float dock_blind_land_alt_m;
    float home_lost_timeout_s;
    float docking_timeout_s;
    float dock_contact_confirm_s;
    float estimator_lost_timeout_s;
    float self_check_s;
    float docked_launch_delay_s;
} MissionConfig;

typedef struct {
    NavState nav;
    TargetTrack target;
    HomeTrack home;
    uint8_t impact_detected;
    uint8_t start_command;
    uint8_t time_exceeded;
    uint8_t return_required;
    uint8_t emergency_requested;
    uint8_t nav_failure;
    uint8_t geofence_violation;
    uint8_t recovery_managed;
    uint8_t recovery_ready_for_breakaway;
    uint8_t recovery_complete;
    uint8_t recovery_failed;
    uint8_t dock_contact;
    uint8_t charging_detected;
    uint8_t wireless_charge_ready;
    float dt;
} MissionInput;

typedef struct {
    MissionState state;
    GuidanceMode guidance;
    Waypoint current_waypoint;
    Vec3f hold_pos;
    float home_search_alt;
    ReturnStage return_stage;
    HomingStage homing_stage;
    DockingStage docking_stage;
    MissionTransitionReason transition_reason;
    uint8_t state_changed;
    uint8_t mission_complete;
    uint8_t mission_failed;
} MissionOutput;

typedef struct {
    MissionConfig cfg;
    Vec3f home_pos;
    MissionState state;
    MissionState resume_state;
    ReturnStage return_stage;
    HomingStage homing_stage;
    DockingStage docking_stage;
    MissionTransitionReason transition_reason;
    float state_time;
    float target_visible_time;
    float target_lost_time;
    float estimator_lost_time;
    float dock_contact_time;
    float breakaway_speed;
    WaypointQueue outbound;
    WaypointQueue search;
    Vec3f breakaway_target;
    uint8_t recovery_managed;
    uint8_t mission_complete;
    uint8_t mission_failed;
} MissionFsm;

void mission_fsm_init(MissionFsm *fsm,
                      const MissionConfig *cfg,
                      Vec3f home_pos,
                      const WaypointQueue *outbound,
                      const WaypointQueue *search);
void mission_fsm_update(MissionFsm *fsm, const MissionInput *input,
                        MissionOutput *output);
const char *mission_state_name(MissionState state);
const char *return_stage_name(ReturnStage stage);
const char *homing_stage_name(HomingStage stage);
const char *docking_stage_name(DockingStage stage);

#ifdef __cplusplus
}
#endif

#endif /* MISSION_FSM_H */
