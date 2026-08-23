#include <string.h>

#include "nav_runtime.h"

#define NAV_SWARM_FUTURE_POINT_COUNT 8u
#define NAV_TIMESTAMP_HALF_RANGE 0x80000000u

static uint8_t positive_finite(float value)
{
    return (nav_isfinite(value) && value > 0.0f) ? 1u : 0u;
}

static uint8_t nonnegative_finite(float value)
{
    return (nav_isfinite(value) && value >= 0.0f) ? 1u : 0u;
}

static uint8_t camera_valid(const CameraModel *camera)
{
    uint8_t index;
    if (!positive_finite(camera->fx) || !positive_finite(camera->fy) ||
        !positive_finite(camera->width) || !positive_finite(camera->height) ||
        !nav_isfinite(camera->cx) || !nav_isfinite(camera->cy) ||
        camera->cx < 0.0f || camera->cx > camera->width ||
        camera->cy < 0.0f || camera->cy > camera->height) {
        return 0u;
    }
    for (index = 0u; index < 9u; index++) {
        if (!nav_isfinite(camera->m[index])) return 0u;
    }
    return 1u;
}

static uint8_t route_valid(const WaypointQueue *route)
{
    uint8_t index;
    if (route->count == 0u || route->count > WP_QUEUE_MAX ||
        route->index > route->count) {
        return 0u;
    }
    for (index = 0u; index < route->count; index++) {
        if (!vec3_is_finite(route->items[index].pos) ||
            !positive_finite(route->items[index].speed)) {
            return 0u;
        }
    }
    return 1u;
}

static uint8_t quaternion_finite(Quatf attitude)
{
    return (nav_isfinite(attitude.w) && nav_isfinite(attitude.x) &&
            nav_isfinite(attitude.y) && nav_isfinite(attitude.z)) ? 1u : 0u;
}

static uint8_t flow_frame_finite(const FlowFrame *frame)
{
    uint8_t index;
    if (frame->count > VF_MAX_FEATURES) return 0u;
    for (index = 0u; index < frame->count; index++) {
        if (!nav_isfinite(frame->feats[index].u) ||
            !nav_isfinite(frame->feats[index].v)) {
            return 0u;
        }
    }
    return 1u;
}

static uint8_t odometry_finite(const OdomSample *odometry)
{
    if (!odometry->valid) return 1u;
    return (vec3_is_finite(odometry->pos) && vec3_is_finite(odometry->vel) &&
            nav_isfinite(odometry->yaw) && nav_isfinite(odometry->yaw_rate) &&
            quaternion_finite(odometry->att)) ? 1u : 0u;
}

static uint8_t pixel_observation_finite(const PixelObs *pixel)
{
    if (!pixel->visible) return 1u;
    return (nav_isfinite(pixel->u) && nav_isfinite(pixel->v) &&
            positive_finite(pixel->size_px)) ? 1u : 0u;
}

static uint8_t source_index(uint32_t source_mask)
{
    switch (source_mask) {
    case NAV_INPUT_SOURCE_IMU: return 0u;
    case NAV_INPUT_SOURCE_FLOW: return 1u;
    case NAV_INPUT_SOURCE_ODOMETRY: return 2u;
    case NAV_INPUT_SOURCE_TARGET: return 3u;
    case NAV_INPUT_SOURCE_HOME: return 4u;
    default: return 0u;
    }
}

static void log_input_fault(NavRuntime *runtime, uint32_t now_ms,
                            NavLogEventCode code, uint32_t source_mask,
                            uint32_t magnitude, uint32_t *event_flags)
{
    nav_event_log_push(&runtime->event_log, now_ms, code, source_mask,
                       (float)magnitude);
    *event_flags |= NAV_EVENT_INPUT_REJECTED;
}

static uint8_t source_timestamp_valid(NavRuntime *runtime,
                                      uint32_t source_mask,
                                      uint32_t sample_timestamp_ms,
                                      uint32_t now_ms,
                                      uint32_t max_age_ms,
                                      NavRuntimeHealth *health,
                                      uint32_t *event_flags)
{
    uint8_t index = source_index(source_mask);
    uint8_t valid = 1u;
    uint32_t age = now_ms - sample_timestamp_ms;

    if (age < NAV_TIMESTAMP_HALF_RANGE) {
        if (age > max_age_ms) {
            health->stale_source_mask |= source_mask;
            log_input_fault(runtime, now_ms, NAV_LOG_INPUT_STALE,
                            source_mask, age, event_flags);
            valid = 0u;
        }
    } else {
        uint32_t future = sample_timestamp_ms - now_ms;
        if (future > runtime->cfg.health.future_tolerance_ms) {
            health->out_of_order_source_mask |= source_mask;
            log_input_fault(runtime, now_ms, NAV_LOG_INPUT_OUT_OF_ORDER,
                            source_mask, future, event_flags);
            valid = 0u;
        }
    }

    if ((runtime->source_seen_mask & source_mask) != 0u) {
        uint32_t advance = sample_timestamp_ms -
                           runtime->last_source_timestamp_ms[index];
        if (advance == 0u) {
            health->duplicate_source_mask |= source_mask;
            log_input_fault(runtime, now_ms, NAV_LOG_INPUT_DUPLICATE,
                            source_mask, 0u, event_flags);
            valid = 0u;
        } else if (advance >= NAV_TIMESTAMP_HALF_RANGE) {
            health->out_of_order_source_mask |= source_mask;
            log_input_fault(runtime, now_ms, NAV_LOG_INPUT_OUT_OF_ORDER,
                            source_mask, advance, event_flags);
            valid = 0u;
        }
    }

    if (valid) {
        runtime->source_seen_mask |= source_mask;
        runtime->last_source_timestamp_ms[index] = sample_timestamp_ms;
    }
    return valid;
}

static void mark_nonfinite(NavRuntime *runtime, uint32_t now_ms,
                           uint32_t source_mask, NavRuntimeHealth *health,
                           uint32_t *event_flags)
{
    health->nonfinite_source_mask |= source_mask;
    log_input_fault(runtime, now_ms, NAV_LOG_INPUT_NONFINITE,
                    source_mask, 0u, event_flags);
}

static void mark_invalid(NavRuntime *runtime, uint32_t now_ms,
                         uint32_t source_mask, NavRuntimeHealth *health,
                         uint32_t *event_flags)
{
    health->invalid_source_mask |= source_mask;
    log_input_fault(runtime, now_ms, NAV_LOG_INPUT_INVALID,
                    source_mask, 0u, event_flags);
}

static void watchdog_update(NavRuntime *runtime, uint32_t now_ms,
                            NavRuntimeHealth *health, uint32_t *event_flags)
{
    if (runtime->update_timestamp_seen) {
        uint32_t gap = now_ms - runtime->last_update_timestamp_ms;
        health->cycle_gap_ms = gap;
        if (gap == 0u || gap >= NAV_TIMESTAMP_HALF_RANGE) {
            if (!runtime->watchdog_tripped) {
                nav_event_log_push(&runtime->event_log, now_ms,
                                   NAV_LOG_WATCHDOG_TRIPPED, gap, (float)gap);
            }
            runtime->watchdog_tripped = 1u;
            *event_flags |= NAV_EVENT_WATCHDOG_TRIPPED;
        } else {
            runtime->last_update_timestamp_ms = now_ms;
            if (gap > runtime->cfg.health.max_cycle_gap_ms) {
                if (runtime->watchdog_overruns < 255u) {
                    runtime->watchdog_overruns++;
                }
                nav_event_log_push(&runtime->event_log, now_ms,
                                   NAV_LOG_WATCHDOG_OVERRUN, gap, (float)gap);
                *event_flags |= NAV_EVENT_WATCHDOG_OVERRUN;
                if (runtime->watchdog_overruns >=
                    runtime->cfg.health.watchdog_trip_after_overruns) {
                    if (!runtime->watchdog_tripped) {
                        nav_event_log_push(&runtime->event_log, now_ms,
                                           NAV_LOG_WATCHDOG_TRIPPED,
                                           runtime->watchdog_overruns,
                                           (float)gap);
                    }
                    runtime->watchdog_tripped = 1u;
                    *event_flags |= NAV_EVENT_WATCHDOG_TRIPPED;
                }
            } else {
                runtime->watchdog_overruns = 0u;
            }
        }
    } else {
        runtime->last_update_timestamp_ms = now_ms;
        runtime->update_timestamp_seen = 1u;
    }
    health->consecutive_overruns = runtime->watchdog_overruns;
    health->watchdog_tripped = runtime->watchdog_tripped;
}

static TrajectoryPlanReport trajectory_report_empty(void)
{
    TrajectoryPlanReport report;
    report.requested_backend = TRAJ_BACKEND_OPTIMIZED;
    report.used_backend = TRAJ_BACKEND_OPTIMIZED;
    report.fallback_used = 0u;
    report.detour_used = 0u;
    report.iterations = 0u;
    report.valid = 1u;
    report.check.flags = TRAJ_CHECK_OK;
    report.check.peak_velocity_mps = 0.0f;
    report.check.peak_acceleration_mps2 = 0.0f;
    report.check.minimum_clearance_m = 1e9f;
    report.check.checked_duration_s = 0.0f;
    return report;
}

static uint8_t controller_saturated(const CtrlOutput *control,
                                    const PosCtrlParams *params)
{
    return (vec3_norm(control->accel_cmd) >= 0.98f * params->max_accel ||
            fabsf(control->yaw_rate_cmd) >= 0.98f * params->max_yaw_rate) ? 1u : 0u;
}

static void fill_recovery_guidance(const ImpactRecoveryOutput *recovery,
                                   const NavState *nav,
                                   GuidanceOutput *output)
{
    guidance_output_hold(output, nav);
    output->pos_sp = recovery->hold_position;
    output->vel_sp = recovery->desired_velocity;
    output->use_pos_sp = recovery->use_position;
}

static void sample_future_points(Vec3f start, Vec3f velocity, float dt,
                                 Vec3f *points, uint8_t count)
{
    uint8_t index;
    for (index = 0u; index < count; index++) {
        points[index] = vec3_add(start, vec3_scale(velocity, dt * (float)index));
    }
}

void nav_runtime_config_default(NavRuntimeConfig *cfg)
{
    memset(cfg, 0, sizeof(*cfg));

    cfg->home_pos = vec3_zero();
    wq_init(&cfg->outbound_route);
    (void)wq_push(&cfg->outbound_route, vec3(2.0f, 0.0f, 1.2f), 1.8f);
    (void)wq_push(&cfg->outbound_route, vec3(3.5f, 0.5f, 1.2f), 1.8f);
    wq_init(&cfg->search_route);
    (void)wq_push(&cfg->search_route, vec3(3.5f, 1.5f, 1.2f), 1.2f);
    (void)wq_push(&cfg->search_route, vec3(4.5f, 1.5f, 1.2f), 1.2f);
    (void)wq_push(&cfg->search_route, vec3(4.5f, 2.5f, 1.2f), 1.2f);
    (void)wq_push(&cfg->search_route, vec3(3.5f, 2.5f, 1.2f), 1.2f);

    cfg->mission.takeoff_alt_m = 1.2f;
    cfg->mission.waypoint_tol_m = 0.15f;
    cfg->mission.cruise_speed_mps = 1.8f;
    cfg->mission.search_yaw_rate_rps = 1.2f;
    cfg->mission.target_confirm_s = 0.3f;
    cfg->mission.target_acquire_timeout_s = 0.6f;
    cfg->mission.terminal_range_m = 1.5f;
    cfg->mission.target_lost_timeout_s = 1.0f;
    cfg->mission.recovery_hold_s = 1.2f;
    cfg->mission.recovery_tilt_ok_rad = 0.30f;
    cfg->mission.breakaway_height_m = 0.5f;
    cfg->mission.home_region_tol_m = 0.4f;
    cfg->mission.home_approach_radius_m = 1.5f;
    cfg->mission.home_search_alt_m = 0.8f;
    cfg->mission.homing_fine_radius_m = 0.30f;
    cfg->mission.homing_yaw_tol_rad = 0.30f;
    cfg->mission.dock_alt_m = 0.08f;
    cfg->mission.dock_lateral_tol_m = 0.08f;
    cfg->mission.dock_capture_tol_m = 0.15f;
    cfg->mission.dock_capture_max_alt_m = 0.25f;
    cfg->mission.dock_land_vel_max = 0.10f;
    cfg->mission.dock_blind_land_alt_m = 0.35f;
    cfg->mission.home_lost_timeout_s = 0.5f;
    cfg->mission.docking_timeout_s = 8.0f;
    cfg->mission.dock_contact_confirm_s = 0.15f;
    cfg->mission.estimator_lost_timeout_s = 3.0f;
    cfg->mission.self_check_s = 0.3f;
    cfg->mission.docked_launch_delay_s = 0.5f;

    cfg->estimator.mode = EST_MODE_INS;
    cfg->estimator.vo_degraded_after_s = 0.6f;
    cfg->estimator.vo_lost_after_s = 2.0f;
    cfg->estimator.recovering_hold_s = 0.5f;
    cfg->estimator.lost_timeout_s = 1.5f;
    cfg->estimator.impact_blind_s = 0.4f;
    cfg->estimator.lost_on_impact = 0u;
    cfg->estimator.kp_tilt = 2.0f;
    cfg->estimator.ki_gyro_bias = 0.05f;
    cfg->estimator.kp_vo_pos = 2.0f;
    cfg->estimator.kp_vo_vel = 3.0f;
    cfg->estimator.kp_vo_yaw = 0.0f;
    cfg->estimator.kp_tof = 2.0f;
    cfg->estimator.kp_tof_vel = 8.0f;

    cfg->impact.accel_spike_threshold = 12.0f;
    cfg->impact.gyro_spike_threshold = 12.0f;
    cfg->impact.confirm_samples = 1u;
    impact_detector_default_fusion_config(&cfg->impact_fusion);
    impact_recovery_default_config(&cfg->recovery);

    cfg->safety.max_mission_time_s = 30.0f;
    cfg->safety.soft_return_deadline_s = 24.0f;
    cfg->safety.hard_return_deadline_s = 30.0f;
    cfg->safety.geofence_radius_m = 12.0f;
    cfg->safety.geofence_min_alt_m = -0.20f;
    cfg->safety.geofence_max_alt_m = 3.0f;
    cfg->safety.min_estimator_quality = 0.20f;
    cfg->safety.controller_saturation_timeout_s = 2.5f;
    cfg->safety.collision_critical_timeout_s = 0.75f;
    cfg->safety.trajectory_invalid_timeout_s = 0.25f;

    cfg->ctrl.kp_pos = 2.0f;
    cfg->ctrl.kp_vel = 3.0f;
    cfg->ctrl.max_vel = 2.0f;
    cfg->ctrl.max_accel = 6.0f;
    cfg->ctrl.kp_yaw = 3.0f;
    cfg->ctrl.max_yaw_rate = 2.0f;

    cfg->terminal.approach_speed = 1.2f;
    cfg->terminal.kp = 1.5f;
    cfg->terminal.min_closing_speed = 0.2f;
    cfg->terminal.closing_range_m = 2.4f;
    cfg->terminal.final_align_range_m = 1.0f;
    cfg->terminal.contact_range_m = 0.35f;
    cfg->terminal.closing_speed_mps = 0.9f;
    cfg->terminal.final_speed_mps = 0.55f;
    cfg->terminal.contact_speed_mps = 0.35f;
    cfg->terminal.kp_vertical = 1.2f;
    cfg->terminal.max_vertical_speed_mps = 0.6f;
    cfg->terminal.max_accel_mps2 = 3.0f;
    cfg->terminal.yaw_align_tolerance_rad = 0.35f;
    cfg->terminal.min_confidence = 0.15f;
    cfg->terminal.loss_grace_s = 0.35f;
    cfg->terminal.reacquire_timeout_s = 1.0f;

    cfg->home_guidance.search_alt = 0.8f;
    cfg->home_guidance.descend_speed = 0.45f;
    cfg->home_guidance.kp_lateral = 1.0f;
    cfg->home_guidance.max_lateral_speed = 0.4f;
    cfg->home_guidance.lateral_tol = 0.10f;
    cfg->home_guidance.spiral_rate = 0.25f;
    cfg->home_guidance.spiral_max_radius = 1.5f;
    cfg->home_guidance.spiral_omega = 2.0f;
    cfg->home_guidance.blind_land_alt = 0.35f;
    cfg->home_guidance.blind_land_timeout = 1.0f;
    cfg->home_guidance.kp_yaw = 0.8f;
    cfg->home_guidance.yaw_tolerance_rad = 0.30f;

    trajectory_planner_default_config(&cfg->planner);
    obstacle_avoidance_default_config(&cfg->obstacle_avoidance);
    collision_init(&cfg->swarm_collision, 0.60f);
    swarm_avoidance_default_config(&cfg->swarm_avoidance);

    camera_init(&cfg->cam_forward, 180.0f, 180.0f, 160.0f, 120.0f,
                320.0f, 240.0f, CAM_MOUNT_FORWARD);
    camera_init(&cfg->cam_down, 180.0f, 180.0f, 160.0f, 120.0f,
                320.0f, 240.0f, CAM_MOUNT_DOWN);
    cfg->target_size_m = 0.30f;
    cfg->marker_size_m = 0.25f;
    cfg->flow.min_height = 0.05f;
    cfg->flow.max_height = 5.0f;
    cfg->flow.min_features = 4u;
    cfg->flow.outlier_residual_px = 3.0f;
    cfg->target_filter_cutoff_hz = 5.0f;
    cfg->home_filter_cutoff_hz = 5.0f;
    cfg->nominal_dt_s = 0.01f;
    cfg->home_yaw_rad = 0.0f;
    cfg->relocalization_cooldown_s = 1.0f;
    cfg->relocalization_min_correction_m = 0.05f;
    cfg->health.imu_max_age_ms = 30u;
    cfg->health.flow_max_age_ms = 150u;
    cfg->health.odometry_max_age_ms = 150u;
    cfg->health.vision_max_age_ms = 250u;
    cfg->health.future_tolerance_ms = 5u;
    cfg->health.max_cycle_gap_ms = 50u;
    cfg->health.watchdog_trip_after_overruns = 2u;
    cfg->self_agent_id = 2u;
}

uint32_t nav_runtime_config_validate(const NavRuntimeConfig *cfg)
{
    uint32_t errors = NAV_CONFIG_ERROR_NONE;
    const MissionConfig *mission;
    const TrajectoryLimits *limits;

    if (cfg == 0) return NAV_CONFIG_ERROR_ARGUMENT;
    mission = &cfg->mission;
    limits = &cfg->planner.limits;

    if (!positive_finite(cfg->nominal_dt_s) || cfg->nominal_dt_s > 0.2f ||
        !positive_finite(cfg->target_filter_cutoff_hz) ||
        !positive_finite(cfg->home_filter_cutoff_hz) ||
        !nonnegative_finite(cfg->relocalization_cooldown_s) ||
        !nonnegative_finite(cfg->relocalization_min_correction_m) ||
        cfg->health.imu_max_age_ms == 0u ||
        cfg->health.flow_max_age_ms == 0u ||
        cfg->health.odometry_max_age_ms == 0u ||
        cfg->health.vision_max_age_ms == 0u ||
        cfg->health.max_cycle_gap_ms == 0u ||
        (float)cfg->health.max_cycle_gap_ms < 1000.0f * cfg->nominal_dt_s ||
        cfg->health.future_tolerance_ms > cfg->health.max_cycle_gap_ms ||
        cfg->health.watchdog_trip_after_overruns == 0u) {
        errors |= NAV_CONFIG_ERROR_TIMING;
    }
    if (!positive_finite(mission->takeoff_alt_m) ||
        !positive_finite(mission->waypoint_tol_m) ||
        !positive_finite(mission->cruise_speed_mps) ||
        !nonnegative_finite(mission->search_yaw_rate_rps) ||
        !positive_finite(mission->target_confirm_s) ||
        !positive_finite(mission->target_acquire_timeout_s) ||
        !positive_finite(mission->terminal_range_m) ||
        !positive_finite(mission->target_lost_timeout_s) ||
        !positive_finite(mission->home_region_tol_m) ||
        !positive_finite(mission->home_approach_radius_m) ||
        !positive_finite(mission->home_search_alt_m) ||
        !positive_finite(mission->docking_timeout_s) ||
        !positive_finite(mission->estimator_lost_timeout_s) ||
        !nonnegative_finite(mission->self_check_s) ||
        !nonnegative_finite(mission->docked_launch_delay_s) ||
        mission->dock_alt_m > mission->dock_blind_land_alt_m) {
        errors |= NAV_CONFIG_ERROR_MISSION;
    }
    if ((cfg->estimator.mode != EST_MODE_TRUTH && cfg->estimator.mode != EST_MODE_INS) ||
        !positive_finite(cfg->estimator.vo_degraded_after_s) ||
        !positive_finite(cfg->estimator.vo_lost_after_s) ||
        cfg->estimator.vo_degraded_after_s >= cfg->estimator.vo_lost_after_s ||
        !positive_finite(cfg->estimator.lost_timeout_s) ||
        !nonnegative_finite(cfg->estimator.kp_tilt) ||
        !nonnegative_finite(cfg->estimator.kp_vo_pos) ||
        !nonnegative_finite(cfg->estimator.kp_vo_vel) ||
        !nonnegative_finite(cfg->estimator.kp_tof)) {
        errors |= NAV_CONFIG_ERROR_ESTIMATOR;
    }
    if (!positive_finite(cfg->safety.max_mission_time_s) ||
        !positive_finite(cfg->safety.soft_return_deadline_s) ||
        !positive_finite(cfg->safety.hard_return_deadline_s) ||
        cfg->safety.soft_return_deadline_s >= cfg->safety.hard_return_deadline_s ||
        cfg->safety.hard_return_deadline_s > cfg->safety.max_mission_time_s ||
        !positive_finite(cfg->safety.geofence_radius_m) ||
        !nav_isfinite(cfg->safety.geofence_min_alt_m) ||
        !nav_isfinite(cfg->safety.geofence_max_alt_m) ||
        cfg->safety.geofence_min_alt_m >= cfg->safety.geofence_max_alt_m ||
        !nav_isfinite(cfg->safety.min_estimator_quality) ||
        cfg->safety.min_estimator_quality < 0.0f ||
        cfg->safety.min_estimator_quality > 1.0f ||
        !positive_finite(cfg->safety.controller_saturation_timeout_s) ||
        !positive_finite(cfg->safety.collision_critical_timeout_s) ||
        !positive_finite(cfg->safety.trajectory_invalid_timeout_s)) {
        errors |= NAV_CONFIG_ERROR_SAFETY;
    }
    if (!positive_finite(cfg->ctrl.kp_pos) || !positive_finite(cfg->ctrl.kp_vel) ||
        !positive_finite(cfg->ctrl.max_vel) || !positive_finite(cfg->ctrl.max_accel) ||
        !positive_finite(cfg->ctrl.kp_yaw) ||
        !positive_finite(cfg->ctrl.max_yaw_rate)) {
        errors |= NAV_CONFIG_ERROR_CONTROL;
    }
    if (!positive_finite(cfg->terminal.approach_speed) ||
        !positive_finite(cfg->terminal.closing_range_m) ||
        !positive_finite(cfg->terminal.final_align_range_m) ||
        !positive_finite(cfg->terminal.contact_range_m) ||
        cfg->terminal.contact_range_m >= cfg->terminal.final_align_range_m ||
        cfg->terminal.final_align_range_m >= cfg->terminal.closing_range_m ||
        !positive_finite(cfg->terminal.closing_speed_mps) ||
        !positive_finite(cfg->terminal.final_speed_mps) ||
        !positive_finite(cfg->terminal.contact_speed_mps) ||
        !positive_finite(cfg->terminal.max_accel_mps2) ||
        !positive_finite(cfg->home_guidance.descend_speed) ||
        !positive_finite(cfg->home_guidance.max_lateral_speed) ||
        !positive_finite(cfg->home_guidance.blind_land_timeout)) {
        errors |= NAV_CONFIG_ERROR_GUIDANCE;
    }
    if (!positive_finite(limits->max_duration_s) ||
        !positive_finite(limits->max_velocity_mps) ||
        !positive_finite(limits->max_acceleration_mps2) ||
        !positive_finite(limits->sample_dt_s) ||
        !vec3_is_finite(limits->workspace_min) ||
        !vec3_is_finite(limits->workspace_max) ||
        limits->workspace_min.x >= limits->workspace_max.x ||
        limits->workspace_min.y >= limits->workspace_max.y ||
        limits->workspace_min.z >= limits->workspace_max.z ||
        !nonnegative_finite(limits->collision_clearance_m) ||
        !positive_finite(cfg->planner.optimized_initial_time_scale) ||
        !positive_finite(cfg->planner.simple_time_scale) ||
        !positive_finite(cfg->planner.time_stretch_factor) ||
        cfg->planner.max_iterations == 0u) {
        errors |= NAV_CONFIG_ERROR_PLANNER;
    }
    if (!positive_finite(cfg->obstacle_avoidance.vehicle_radius_m) ||
        !positive_finite(cfg->obstacle_avoidance.warning_clearance_m) ||
        !nonnegative_finite(cfg->obstacle_avoidance.emergency_clearance_m) ||
        cfg->obstacle_avoidance.emergency_clearance_m >=
            cfg->obstacle_avoidance.warning_clearance_m ||
        !positive_finite(cfg->obstacle_avoidance.prediction_horizon_s) ||
        !positive_finite(cfg->obstacle_avoidance.max_avoidance_speed_mps) ||
        !positive_finite(cfg->obstacle_avoidance.max_observation_age_s)) {
        errors |= NAV_CONFIG_ERROR_OBSTACLE;
    }
    if (!camera_valid(&cfg->cam_forward) || !camera_valid(&cfg->cam_down) ||
        !positive_finite(cfg->target_size_m) ||
        !positive_finite(cfg->marker_size_m) ||
        !nav_isfinite(cfg->home_yaw_rad) ||
        !positive_finite(cfg->flow.min_height) ||
        !positive_finite(cfg->flow.max_height) ||
        cfg->flow.min_height >= cfg->flow.max_height ||
        cfg->flow.min_features == 0u || cfg->flow.min_features > VF_MAX_FEATURES ||
        !positive_finite(cfg->flow.outlier_residual_px)) {
        errors |= NAV_CONFIG_ERROR_CAMERA;
    }
    if (!route_valid(&cfg->outbound_route) || !route_valid(&cfg->search_route) ||
        !vec3_is_finite(cfg->home_pos)) {
        errors |= NAV_CONFIG_ERROR_ROUTE;
    }
    if (!positive_finite(cfg->swarm_collision.safe_separation_m) ||
        !positive_finite(cfg->swarm_collision.critical_separation_m) ||
        cfg->swarm_collision.critical_separation_m >=
            cfg->swarm_collision.safe_separation_m ||
        !positive_finite(cfg->swarm_collision.prediction_horizon_s) ||
        !positive_finite(cfg->swarm_collision.sample_dt_s) ||
        !positive_finite(cfg->swarm_collision.max_message_age_s) ||
        (cfg->swarm_avoidance.mode != SWARM_DISABLED &&
         cfg->swarm_avoidance.mode != SWARM_ENABLED)) {
        errors |= NAV_CONFIG_ERROR_SWARM;
    }
    return errors;
}

const char *nav_runtime_config_error_name(uint32_t single_error)
{
    switch (single_error) {
    case NAV_CONFIG_ERROR_NONE: return "none";
    case NAV_CONFIG_ERROR_ARGUMENT: return "argument";
    case NAV_CONFIG_ERROR_TIMING: return "timing";
    case NAV_CONFIG_ERROR_MISSION: return "mission";
    case NAV_CONFIG_ERROR_ESTIMATOR: return "estimator";
    case NAV_CONFIG_ERROR_SAFETY: return "safety";
    case NAV_CONFIG_ERROR_CONTROL: return "control";
    case NAV_CONFIG_ERROR_GUIDANCE: return "guidance";
    case NAV_CONFIG_ERROR_PLANNER: return "planner";
    case NAV_CONFIG_ERROR_OBSTACLE: return "obstacle";
    case NAV_CONFIG_ERROR_CAMERA: return "camera";
    case NAV_CONFIG_ERROR_ROUTE: return "route";
    case NAV_CONFIG_ERROR_SWARM: return "swarm";
    default: return "multiple-or-unknown";
    }
}

uint32_t nav_runtime_init(NavRuntime *runtime, const NavRuntimeConfig *cfg)
{
    uint32_t errors;
    if (runtime == 0) return NAV_CONFIG_ERROR_ARGUMENT;
    memset(runtime, 0, sizeof(*runtime));
    errors = nav_runtime_config_validate(cfg);
    runtime->config_errors = errors;
    if (errors != NAV_CONFIG_ERROR_NONE) return errors;

    runtime->cfg = *cfg;
    nav_event_log_init(&runtime->event_log);
    nav_event_log_push(&runtime->event_log, 0u, NAV_LOG_RUNTIME_STARTED,
                       0u, 0.0f);
    estimator_init(&runtime->estimator, &cfg->estimator);
    impact_detector_init(&runtime->impact_detector, &cfg->impact);
    impact_detector_configure_fusion(&runtime->impact_detector, &cfg->impact_fusion);
    impact_recovery_init(&runtime->recovery, &cfg->recovery);
    runtime->recovery_output.stage = RECOVERY_IDLE;
    runtime->recovery_output.hold_position = cfg->home_pos;
    runtime->recovery_output.desired_velocity = vec3_zero();
    runtime->recovery_output.breakaway_target = cfg->home_pos;
    target_tracker_init(&runtime->target_tracker, cfg->target_filter_cutoff_hz,
                        cfg->nominal_dt_s);
    home_detector_init(&runtime->home_detector, cfg->home_filter_cutoff_hz,
                       cfg->nominal_dt_s);
    safety_init(&runtime->safety, &cfg->safety);
    mission_fsm_init(&runtime->mission, &cfg->mission, cfg->home_pos,
                     &cfg->outbound_route, &cfg->search_route);
    terminal_guidance_init(&runtime->terminal, &cfg->terminal);
    vf_init(&runtime->vision_frontend, &cfg->flow, &cfg->cam_down);
    obstacle_set_init(&runtime->obstacles);
    swarm_view_init(&runtime->swarm, cfg->self_agent_id);
    runtime->trajectory_report = trajectory_report_empty();
    runtime->trajectory_valid = 1u;
    runtime->previous_control.accel_cmd = vec3_zero();
    runtime->previous_control.yaw_rate_cmd = 0.0f;
    runtime->mission_output.state = MS_BOOT;
    runtime->mission_output.guidance = GM_NONE;
    runtime->output.trajectory = runtime->trajectory_report;
    runtime->output.trajectory_valid = 1u;
    runtime->initialized = 1u;
    return NAV_CONFIG_ERROR_NONE;
}

uint8_t nav_runtime_step(NavRuntime *runtime, const NavRuntimeInput *input)
{
    const NavRuntimeConfig *cfg;
    const FlowFrame *flow_sample = 0;
    const OdomSample *odometry_sample = 0;
    const PixelObs *target_pixel = 0;
    const PixelObs *home_pixel = 0;
    OdomSample odometry;
    TargetObs target_observation;
    HomeObs home_observation;
    ImpactReport impact_report;
    ObstacleRiskReport obstacle_report;
    CollisionReport swarm_report;
    SafetyInput safety_input;
    SafetyDecision safety_decision;
    MissionInput mission_input;
    GuidanceOutput guidance;
    NavRuntimeHealth health;
    CollisionRiskLevel combined_collision_risk;
    uint32_t event_flags = NAV_EVENT_NONE;
    uint32_t time_ms;
    float dt;

    if (runtime == 0 || !runtime->initialized || input == 0) {
        if (runtime != 0) {
            runtime->output.armed = 0u;
            runtime->output.step_valid = 0u;
            runtime->output.control.accel_cmd = vec3_zero();
            runtime->output.control.yaw_rate_cmd = 0.0f;
        }
        return 0u;
    }
    cfg = &runtime->cfg;
    time_ms = input->timestamp_ms != 0u
        ? input->timestamp_ms
        : (input->imu != 0 ? input->imu->timestamp_ms : 0u);
    memset(&health, 0, sizeof(health));
    watchdog_update(runtime, time_ms, &health, &event_flags);
    if (!positive_finite(input->dt) || input->dt > 0.2f) {
        if (!runtime->watchdog_tripped) {
            nav_event_log_push(&runtime->event_log, time_ms,
                               NAV_LOG_WATCHDOG_TRIPPED, 0u, input->dt);
        }
        runtime->watchdog_tripped = 1u;
        health.watchdog_tripped = 1u;
        event_flags |= NAV_EVENT_WATCHDOG_TRIPPED;
        runtime->output.health = health;
        runtime->output.event_flags = event_flags;
        runtime->output.armed = 0u;
        runtime->output.step_valid = 0u;
        runtime->output.control.accel_cmd = vec3_zero();
        runtime->output.control.yaw_rate_cmd = 0.0f;
        return 0u;
    }
    dt = input->dt;

    if (input->imu == 0) {
        health.stale_source_mask |= NAV_INPUT_SOURCE_IMU;
        log_input_fault(runtime, time_ms, NAV_LOG_INPUT_STALE,
                        NAV_INPUT_SOURCE_IMU, 0u, &event_flags);
    } else {
        uint8_t imu_valid = source_timestamp_valid(runtime,
            NAV_INPUT_SOURCE_IMU, input->imu->timestamp_ms, time_ms,
            cfg->health.imu_max_age_ms, &health, &event_flags);
        if (!vec3_is_finite(input->imu->accel) ||
            !vec3_is_finite(input->imu->gyro)) {
            mark_nonfinite(runtime, time_ms, NAV_INPUT_SOURCE_IMU,
                           &health, &event_flags);
            imu_valid = 0u;
        }
        health.required_input_valid = imu_valid;
    }
    if (!health.required_input_valid) {
        runtime->output.health = health;
        runtime->output.event_flags = event_flags;
        runtime->output.armed = 0u;
        runtime->output.step_valid = 0u;
        runtime->output.control.accel_cmd = vec3_zero();
        runtime->output.control.yaw_rate_cmd = 0.0f;
        return 0u;
    }

    if (input->flow != 0 &&
        source_timestamp_valid(runtime, NAV_INPUT_SOURCE_FLOW,
            input->flow->timestamp_ms, time_ms, cfg->health.flow_max_age_ms,
            &health, &event_flags)) {
        if (flow_frame_finite(input->flow)) {
            flow_sample = input->flow;
        } else {
            mark_nonfinite(runtime, time_ms, NAV_INPUT_SOURCE_FLOW,
                           &health, &event_flags);
        }
    }
    if (input->odometry != 0 &&
        source_timestamp_valid(runtime, NAV_INPUT_SOURCE_ODOMETRY,
            input->odometry->timestamp_ms, time_ms,
            cfg->health.odometry_max_age_ms, &health, &event_flags)) {
        if (odometry_finite(input->odometry)) {
            odometry_sample = input->odometry;
        } else {
            mark_nonfinite(runtime, time_ms, NAV_INPUT_SOURCE_ODOMETRY,
                           &health, &event_flags);
        }
    }
    if (input->target_pixel != 0 &&
        source_timestamp_valid(runtime, NAV_INPUT_SOURCE_TARGET,
            input->target_pixel->timestamp_ms, time_ms,
            cfg->health.vision_max_age_ms, &health, &event_flags)) {
        if (pixel_observation_finite(input->target_pixel)) {
            target_pixel = input->target_pixel;
        } else {
            mark_invalid(runtime, time_ms, NAV_INPUT_SOURCE_TARGET,
                         &health, &event_flags);
        }
    }
    if (input->home_pixel != 0 &&
        source_timestamp_valid(runtime, NAV_INPUT_SOURCE_HOME,
            input->home_pixel->timestamp_ms, time_ms,
            cfg->health.vision_max_age_ms, &health, &event_flags)) {
        if (pixel_observation_finite(input->home_pixel)) {
            home_pixel = input->home_pixel;
        } else {
            mark_invalid(runtime, time_ms, NAV_INPUT_SOURCE_HOME,
                         &health, &event_flags);
        }
    }

    odometry.pos = vec3_zero();
    odometry.vel = vec3_zero();
    odometry.yaw = 0.0f;
    odometry.yaw_rate = 0.0f;
    odometry.att = runtime->estimator.out.att;
    odometry.valid = 0u;
    odometry.timestamp_ms = time_ms;
    if (flow_sample != 0) {
        vf_update(&runtime->vision_frontend, flow_sample, input->imu->gyro,
                  runtime->estimator.out.att, input->tof_height, dt, &odometry);
    } else if (odometry_sample != 0) {
        odometry = *odometry_sample;
    }
    estimator_update(&runtime->estimator, input->imu, &odometry,
                     input->tof_height, dt);

    impact_detector_arm(&runtime->impact_detector,
        (runtime->mission.state == MS_TARGET_TRACK ||
         runtime->mission.state == MS_TERMINAL) ? 1u : 0u);
    impact_report = impact_detector_update_fused(&runtime->impact_detector,
        input->imu, runtime->estimator.out.vel, runtime->estimator.out.att, dt);
    if (impact_report.rising_edge) {
        estimator_notify_impact(&runtime->estimator);
        impact_recovery_start(&runtime->recovery, &runtime->estimator.out);
        event_flags |= NAV_EVENT_IMPACT_CONFIRMED;
        nav_event_log_push(&runtime->event_log, time_ms,
                           NAV_LOG_IMPACT_CONFIRMED, 0u,
                           impact_report.confidence);
    }

    target_observation.visible =
        (target_pixel != 0) ? target_pixel->visible : 0u;
    target_observation.timestamp_ms = target_observation.visible &&
        target_pixel->timestamp_ms != 0u
        ? target_pixel->timestamp_ms : time_ms;
    target_observation.confidence = target_observation.visible ? 1.0f : 0.0f;
    target_observation.rel_pos = target_observation.visible
        ? camera_reconstruct_nav(&cfg->cam_forward, target_pixel->u,
            target_pixel->v, target_pixel->size_px,
            cfg->target_size_m, runtime->estimator.out.att)
        : vec3_zero();
    target_observation.bearing = vec3_normalize_or(target_observation.rel_pos,
                                                   vec3(1.0f, 0.0f, 0.0f));

    home_observation.visible =
        (home_pixel != 0) ? home_pixel->visible : 0u;
    home_observation.timestamp_ms = home_observation.visible &&
        home_pixel->timestamp_ms != 0u
        ? home_pixel->timestamp_ms : time_ms;
    home_observation.confidence = home_observation.visible ? 1.0f : 0.0f;
    home_observation.rel_pos = home_observation.visible
        ? camera_reconstruct_nav(&cfg->cam_down, home_pixel->u,
            home_pixel->v, home_pixel->size_px,
            cfg->marker_size_m, runtime->estimator.out.att)
        : vec3_zero();
    home_observation.relative_yaw = home_observation.visible
        ? wrap_pi(cfg->home_yaw_rad - runtime->estimator.out.yaw) : 0.0f;

    target_tracker_update(&runtime->target_tracker, &target_observation, dt);
    home_detector_update(&runtime->home_detector, &home_observation, dt);

    if (home_observation.visible &&
        (!runtime->relocalization_seen ||
         (uint32_t)(time_ms - runtime->last_relocalization_ms) >=
        (uint32_t)(1000.0f * cfg->relocalization_cooldown_s))) {
        NavState absolute_reference = runtime->estimator.out;
        float correction_distance;
        absolute_reference.pos = vec3_sub(cfg->home_pos, home_observation.rel_pos);
        correction_distance = vec3_dist(absolute_reference.pos,
                                        runtime->estimator.out.pos);
        if (correction_distance >= cfg->relocalization_min_correction_m ||
            runtime->estimator.out.status != EST_TRACKING) {
            estimator_notify_relocalized(&runtime->estimator, &absolute_reference);
            vf_set_pose(&runtime->vision_frontend, absolute_reference.pos,
                        absolute_reference.yaw);
            runtime->last_relocalization_ms = time_ms;
            runtime->relocalization_seen = 1u;
            event_flags |= NAV_EVENT_RELOCALIZED;
            nav_event_log_push(&runtime->event_log, time_ms,
                               NAV_LOG_RELOCALIZED, 0u,
                               correction_distance);
        }
    }

    impact_recovery_update(&runtime->recovery, &runtime->estimator.out,
                           odometry.valid, dt, &runtime->recovery_output);

    obstacle_set_init(&runtime->obstacles);
    if (input->obstacles != 0) {
        runtime->obstacles = *input->obstacles;
        if (runtime->obstacles.count > LOCAL_OBSTACLE_MAX) {
            runtime->obstacles.count = LOCAL_OBSTACLE_MAX;
        }
    }
    obstacle_report = obstacle_evaluate(&cfg->obstacle_avoidance,
        runtime->estimator.out.pos, runtime->estimator.out.vel,
        runtime->estimator.out.vel, &runtime->obstacles);
    if (obstacle_report.level != COLLISION_RISK_NONE) {
        event_flags |= NAV_EVENT_OBSTACLE_RISK;
        if (runtime->output.obstacle.level == COLLISION_RISK_NONE) {
            nav_event_log_push(&runtime->event_log, time_ms,
                               NAV_LOG_OBSTACLE_RISK,
                               obstacle_report.obstacle_id,
                               obstacle_report.minimum_separation_m);
        }
    }

    {
        Vec3f future_points[NAV_SWARM_FUTURE_POINT_COUNT];
        CollisionConfig collision_cfg = cfg->swarm_collision;
        uint16_t sequence = runtime->swarm.self.sequence;
        uint8_t index;
        runtime->swarm.other_count = 0u;
        if (input->swarm != 0) {
            runtime->swarm.other_count = input->swarm->other_count;
            if (runtime->swarm.other_count > SWARM_MAX_OTHER_AGENTS) {
                runtime->swarm.other_count = SWARM_MAX_OTHER_AGENTS;
            }
            for (index = 0u; index < runtime->swarm.other_count; index++) {
                runtime->swarm.others[index] = input->swarm->others[index];
            }
        }
        runtime->swarm.self.agent_id = cfg->self_agent_id;
        runtime->swarm.self.sequence = (uint16_t)(sequence + 1u);
        runtime->swarm.self.timestamp_ms = time_ms;
        runtime->swarm.self.pos = runtime->estimator.out.pos;
        runtime->swarm.self.vel = runtime->estimator.out.vel;
        runtime->swarm.self.quality = runtime->estimator.out.quality;
        runtime->swarm.self.age_s = 0.0f;
        runtime->swarm.self.valid = 1u;

        collision_cfg.sample_dt_s = collision_cfg.prediction_horizon_s /
            (float)(NAV_SWARM_FUTURE_POINT_COUNT - 1u);
        sample_future_points(runtime->estimator.out.pos,
                             runtime->estimator.out.vel,
                             collision_cfg.sample_dt_s, future_points,
                             NAV_SWARM_FUTURE_POINT_COUNT);
        swarm_report = collision_check(&collision_cfg, future_points,
            NAV_SWARM_FUTURE_POINT_COUNT, &runtime->swarm);
    }
    if (swarm_report.conflict) {
        event_flags |= NAV_EVENT_SWARM_CONFLICT;
        if (!runtime->output.swarm_collision.conflict) {
            nav_event_log_push(&runtime->event_log, time_ms,
                               NAV_LOG_SWARM_CONFLICT,
                               swarm_report.other_agent_id,
                               swarm_report.min_separation);
        }
    }

    combined_collision_risk = obstacle_report.level;
    if (swarm_report.risk == SWARM_RISK_CRITICAL) {
        combined_collision_risk = COLLISION_RISK_CRITICAL;
    } else if (swarm_report.risk == SWARM_RISK_WARNING &&
               combined_collision_risk == COLLISION_RISK_NONE) {
        combined_collision_risk = COLLISION_RISK_WARNING;
    }

    safety_input.nav = &runtime->estimator.out;
    safety_input.home_position = cfg->home_pos;
    safety_input.impact_state = impact_report.rising_edge
        ? CONFIRMED_IMPACT : NO_IMPACT;
    safety_input.collision_risk = combined_collision_risk;
    safety_input.trajectory_valid = runtime->trajectory_valid;
    safety_input.controller_saturated = controller_saturated(
        &runtime->previous_control, &cfg->ctrl);
    safety_input.dt = dt;
    safety_decision = safety_update_full(&runtime->safety, &safety_input);

    mission_input.nav = runtime->estimator.out;
    mission_input.target = runtime->target_tracker.out;
    mission_input.home = runtime->home_detector.out;
    mission_input.impact_detected = runtime->impact_detector.triggered;
    mission_input.start_command = input->start_command;
    mission_input.time_exceeded = runtime->safety.time_exceeded;
    mission_input.return_required = (uint8_t)(safety_decision.request_return ||
                                              input->request_return);
    mission_input.emergency_requested = (uint8_t)(safety_decision.request_emergency ||
                                                   input->request_emergency);
    if (runtime->recovery.active &&
        (safety_decision.reason_mask &
         ~(SAFETY_REASON_ESTIMATOR | SAFETY_REASON_IMPACT)) == 0u &&
        !input->request_emergency) {
        mission_input.emergency_requested = 0u;
    }
    mission_input.nav_failure = runtime->watchdog_tripped;
    mission_input.geofence_violation = runtime->safety.geofence_violation;
    mission_input.recovery_managed =
        (runtime->recovery.stage != RECOVERY_IDLE) ? 1u : 0u;
    mission_input.recovery_ready_for_breakaway =
        runtime->recovery_output.ready_for_breakaway;
    mission_input.recovery_complete = runtime->recovery_output.complete;
    mission_input.recovery_failed = runtime->recovery_output.failed;
    mission_input.dock_contact = input->dock_contact;
    mission_input.charging_detected = input->charging_detected;
    mission_input.wireless_charge_ready = input->wireless_charge_ready;
    mission_input.dt = dt;
    mission_fsm_update(&runtime->mission, &mission_input,
                       &runtime->mission_output);

    if (runtime->mission_output.state_changed) {
        uint32_t transition =
            ((uint32_t)runtime->output.mission.state << 8) |
            (uint32_t)runtime->mission_output.state;
        nav_event_log_push(&runtime->event_log, time_ms,
                           NAV_LOG_MISSION_TRANSITION, transition,
                           runtime->mission.state_time);
    }

    if (runtime->mission_output.state_changed &&
        (runtime->mission_output.state == MS_TARGET_ACQUIRE ||
         runtime->mission_output.state == MS_TARGET_TRACK)) {
        terminal_guidance_reset(&runtime->terminal);
        impact_detector_reset(&runtime->impact_detector);
    }

    if (runtime->mission_output.guidance != GM_WAYPOINT) {
        runtime->trajectory_active = 0u;
        runtime->trajectory_valid = 1u;
    }
    switch (runtime->mission_output.guidance) {
    case GM_TAKEOFF:
        guidance_takeoff(&runtime->mission_output.hold_pos,
                         &runtime->estimator.out, &guidance);
        break;
    case GM_HOLD:
        guidance_hold(&runtime->mission_output.hold_pos,
                      &runtime->estimator.out, &guidance);
        break;
    case GM_WAYPOINT:
        if (!runtime->trajectory_active || runtime->mission_output.state_changed ||
            traj_done(&runtime->trajectory, runtime->trajectory_time_s)) {
            runtime->trajectory_report = trajectory_plan_single(&runtime->trajectory,
                runtime->estimator.out.pos, runtime->estimator.out.vel,
                runtime->mission_output.current_waypoint.pos,
                runtime->mission_output.current_waypoint.speed > 0.1f
                    ? runtime->mission_output.current_waypoint.speed
                    : cfg->mission.cruise_speed_mps,
                &cfg->planner, &runtime->obstacles, TRAJ_BACKEND_OPTIMIZED);
            runtime->trajectory_time_s = 0.0f;
            runtime->trajectory_active = runtime->trajectory_report.valid;
            runtime->trajectory_valid = runtime->trajectory_report.valid;
            if (runtime->trajectory_report.detour_used) {
                event_flags |= NAV_EVENT_TRAJECTORY_DETOUR;
                nav_event_log_push(&runtime->event_log, time_ms,
                                   NAV_LOG_TRAJECTORY_DETOUR,
                                   runtime->trajectory_report.iterations,
                                   runtime->trajectory.duration);
            }
            if (!runtime->trajectory_report.valid &&
                (runtime->trajectory_report.check.flags &
                 ~(TRAJ_CHECK_VELOCITY | TRAJ_CHECK_ACCELERATION)) == 0u) {
                /* The direct waypoint law remains bounded by the controller.
                 * Only pure kinematic post-check failures may use it without
                 * escalating to a trajectory-integrity safety fault. */
                runtime->trajectory_valid = 1u;
            } else if (!runtime->trajectory_report.valid) {
                event_flags |= NAV_EVENT_TRAJECTORY_INVALID;
                nav_event_log_push(&runtime->event_log, time_ms,
                                   NAV_LOG_TRAJECTORY_INVALID,
                                   runtime->trajectory_report.check.flags,
                                   runtime->trajectory_report.check.checked_duration_s);
            }
        }
        if (runtime->trajectory_active) {
            trajectory_guidance_update(&runtime->trajectory,
                runtime->trajectory_time_s, &runtime->estimator.out, &guidance);
            runtime->trajectory_time_s += dt;
        } else {
            cruise_guidance_update(&runtime->mission_output.current_waypoint,
                                   &runtime->estimator.out, &guidance);
        }
        if (runtime->mission_output.state == MS_SEARCH) {
            guidance.yaw_sp = wrap_pi((float)time_ms * 0.001f *
                                      cfg->mission.search_yaw_rate_rps);
        }
        break;
    case GM_TERMINAL:
        terminal_guidance_update(&runtime->terminal, &runtime->target_tracker.out,
                                 &runtime->estimator.out, dt, &guidance);
        break;
    case GM_RECOVERY:
        if (runtime->recovery.stage != RECOVERY_IDLE) {
            fill_recovery_guidance(&runtime->recovery_output,
                                   &runtime->estimator.out, &guidance);
        } else {
            recovery_guidance_update(&runtime->estimator.out,
                                     cfg->recovery.damping_gain, &guidance);
        }
        break;
    case GM_HOME_SERVO:
        home_guidance_update(&runtime->home_detector.out, &runtime->estimator.out,
                             &cfg->home_pos, &cfg->home_guidance, &guidance);
        break;
    case GM_LAND:
        guidance_land(&runtime->estimator.out, 0.3f, &guidance);
        break;
    case GM_NONE:
    default:
        guidance_output_hold(&guidance, &runtime->estimator.out);
        guidance.use_pos_sp = 0u;
        break;
    }

    obstacle_report = obstacle_evaluate(&cfg->obstacle_avoidance,
        runtime->estimator.out.pos, runtime->estimator.out.vel,
        guidance.vel_sp, &runtime->obstacles);
    guidance.vel_sp = obstacle_apply_avoidance(guidance.vel_sp,
        &obstacle_report, cfg->ctrl.max_vel);
    if (swarm_report.conflict) {
        SwarmAvoidanceDecision avoidance = swarm_avoidance_decide(
            &cfg->swarm_avoidance, cfg->self_agent_id,
            &swarm_report, guidance.vel_sp);
        guidance.vel_sp = vec3_clamp_norm(vec3_add(guidance.vel_sp,
            avoidance.velocity_bias), cfg->ctrl.max_vel);
    }

    pos_controller_update(&cfg->ctrl, &guidance,
                          &runtime->estimator.out, &runtime->previous_control);

    runtime->output.nav = runtime->estimator.out;
    runtime->output.mission = runtime->mission_output;
    runtime->output.safety = safety_decision;
    runtime->output.impact = impact_report;
    runtime->output.recovery = runtime->recovery_output;
    runtime->output.terminal_stage = runtime->terminal.stage;
    runtime->output.target = runtime->target_tracker.out;
    runtime->output.home = runtime->home_detector.out;
    runtime->output.obstacle = obstacle_report;
    runtime->output.swarm_collision = swarm_report;
    runtime->output.trajectory = runtime->trajectory_report;
    runtime->output.guidance = guidance;
    runtime->output.control = runtime->previous_control;
    runtime->output.swarm_self = runtime->swarm.self;
    runtime->output.health = health;
    runtime->output.event_flags = event_flags;
    runtime->output.recovery_active = runtime->recovery.active;
    runtime->output.trajectory_active = runtime->trajectory_active;
    runtime->output.trajectory_valid = runtime->trajectory_valid;
    runtime->output.armed =
        (runtime->mission_output.state != MS_BOOT &&
         runtime->mission_output.state != MS_SELF_CHECK &&
         runtime->mission_output.state != MS_DOCKED) ? 1u : 0u;
    runtime->output.step_valid = 1u;
    return 1u;
}

const NavEventLog *nav_runtime_event_log(const NavRuntime *runtime)
{
    return runtime != 0 ? &runtime->event_log : 0;
}
