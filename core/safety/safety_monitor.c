#include "safety_monitor.h"

void safety_init(SafetyMonitor *monitor, const SafetyConfig *cfg)
{
    monitor->cfg = *cfg;
    if (monitor->cfg.soft_return_deadline_s <= 0.0f) {
        monitor->cfg.soft_return_deadline_s = 0.78f * monitor->cfg.max_mission_time_s;
    }
    if (monitor->cfg.hard_return_deadline_s <= 0.0f) {
        monitor->cfg.hard_return_deadline_s = monitor->cfg.max_mission_time_s;
    }
    if (monitor->cfg.min_estimator_quality <= 0.0f) {
        monitor->cfg.min_estimator_quality = 0.25f;
    }
    if (!nav_isfinite(monitor->cfg.geofence_min_alt_m) ||
        monitor->cfg.geofence_min_alt_m >= 0.0f) {
        monitor->cfg.geofence_min_alt_m = -0.20f;
    }
    if (monitor->cfg.controller_saturation_timeout_s <= 0.0f) {
        monitor->cfg.controller_saturation_timeout_s = 0.75f;
    }
    if (monitor->cfg.collision_critical_timeout_s <= 0.0f) {
        monitor->cfg.collision_critical_timeout_s = 0.75f;
    }
    if (monitor->cfg.trajectory_invalid_timeout_s <= 0.0f) {
        monitor->cfg.trajectory_invalid_timeout_s = 0.25f;
    }
    monitor->elapsed_s = 0.0f;
    monitor->remaining_s = monitor->cfg.max_mission_time_s;
    monitor->controller_saturation_time_s = 0.0f;
    monitor->collision_critical_time_s = 0.0f;
    monitor->trajectory_invalid_time_s = 0.0f;
    monitor->time_exceeded = 0u;
    monitor->soft_return_reached = 0u;
    monitor->hard_return_reached = 0u;
    monitor->geofence_violation = 0u;
    monitor->decision.level = SAFETY_SAFE;
    monitor->decision.reason_mask = SAFETY_REASON_NONE;
    monitor->decision.elapsed_s = 0.0f;
    monitor->decision.remaining_s = monitor->remaining_s;
    monitor->decision.request_return = 0u;
    monitor->decision.request_recovery = 0u;
    monitor->decision.request_emergency = 0u;
}

SafetyDecision safety_update_full(SafetyMonitor *monitor,
                                  const SafetyInput *input)
{
    float step = (nav_isfinite(input->dt) && input->dt > 0.0f && input->dt <= 0.2f)
        ? input->dt : 0.01f;
    SafetyDecision decision;

    monitor->elapsed_s += step;
    monitor->remaining_s = clampf(monitor->cfg.max_mission_time_s - monitor->elapsed_s,
                                  0.0f, monitor->cfg.max_mission_time_s);
    monitor->soft_return_reached =
        (monitor->elapsed_s >= monitor->cfg.soft_return_deadline_s) ? 1u : 0u;
    monitor->hard_return_reached =
        (monitor->elapsed_s >= monitor->cfg.hard_return_deadline_s) ? 1u : 0u;
    monitor->time_exceeded = monitor->hard_return_reached;
    monitor->geofence_violation =
        (vec3_dist_xy(input->nav->pos, input->home_position) > monitor->cfg.geofence_radius_m ||
         input->nav->pos.z > monitor->cfg.geofence_max_alt_m ||
         input->nav->pos.z < monitor->cfg.geofence_min_alt_m) ? 1u : 0u;

    monitor->controller_saturation_time_s = input->controller_saturated
        ? monitor->controller_saturation_time_s + step : 0.0f;
    monitor->collision_critical_time_s =
        (input->collision_risk == COLLISION_RISK_CRITICAL)
        ? monitor->collision_critical_time_s + step : 0.0f;
    monitor->trajectory_invalid_time_s = input->trajectory_valid
        ? 0.0f : monitor->trajectory_invalid_time_s + step;

    decision.level = SAFETY_SAFE;
    decision.reason_mask = SAFETY_REASON_NONE;
    decision.elapsed_s = monitor->elapsed_s;
    decision.remaining_s = monitor->remaining_s;
    decision.request_return = 0u;
    decision.request_recovery = 0u;
    decision.request_emergency = 0u;

    if (monitor->soft_return_reached) {
        decision.level = SAFETY_RETURN_REQUIRED;
        decision.reason_mask |= SAFETY_REASON_SOFT_DEADLINE;
        decision.request_return = 1u;
    }
    if (input->nav->status == EST_DEGRADED || input->nav->quality < monitor->cfg.min_estimator_quality ||
        input->collision_risk != COLLISION_RISK_NONE) {
        if (decision.level < SAFETY_DEGRADED) {
            decision.level = SAFETY_DEGRADED;
        }
        if (input->nav->status == EST_DEGRADED ||
            input->nav->quality < monitor->cfg.min_estimator_quality) {
            decision.reason_mask |= SAFETY_REASON_ESTIMATOR;
        }
        if (input->collision_risk != COLLISION_RISK_NONE) {
            decision.reason_mask |= SAFETY_REASON_COLLISION;
        }
    }
    if (input->impact_state == CONFIRMED_IMPACT) {
        decision.level = SAFETY_RECOVERY_REQUIRED;
        decision.reason_mask |= SAFETY_REASON_IMPACT;
        decision.request_recovery = 1u;
    }
    if (monitor->trajectory_invalid_time_s >= monitor->cfg.trajectory_invalid_timeout_s) {
        decision.level = SAFETY_RETURN_REQUIRED;
        decision.reason_mask |= SAFETY_REASON_TRAJECTORY;
        decision.request_return = 1u;
    }
    if (monitor->hard_return_reached || monitor->geofence_violation ||
        input->nav->status == EST_LOST ||
        monitor->collision_critical_time_s >= monitor->cfg.collision_critical_timeout_s ||
        monitor->controller_saturation_time_s >= monitor->cfg.controller_saturation_timeout_s) {
        decision.level = SAFETY_EMERGENCY;
        decision.request_emergency = 1u;
        if (monitor->hard_return_reached) decision.reason_mask |= SAFETY_REASON_HARD_DEADLINE;
        if (monitor->geofence_violation) decision.reason_mask |= SAFETY_REASON_GEOFENCE;
        if (input->nav->status == EST_LOST) decision.reason_mask |= SAFETY_REASON_ESTIMATOR;
        if (monitor->collision_critical_time_s >= monitor->cfg.collision_critical_timeout_s)
            decision.reason_mask |= SAFETY_REASON_COLLISION;
        if (monitor->controller_saturation_time_s >= monitor->cfg.controller_saturation_timeout_s)
            decision.reason_mask |= SAFETY_REASON_CONTROLLER;
    }

    monitor->decision = decision;
    return decision;
}

void safety_update(SafetyMonitor *monitor, const NavState *nav,
                   const Vec3f *home, float dt)
{
    SafetyInput input;
    input.nav = nav;
    input.home_position = *home;
    input.impact_state = NO_IMPACT;
    input.collision_risk = COLLISION_RISK_NONE;
    input.trajectory_valid = 1u;
    input.controller_saturated = 0u;
    input.dt = dt;
    (void)safety_update_full(monitor, &input);
}

const char *safety_level_name(SafetyLevel level)
{
    switch (level) {
    case SAFETY_SAFE:              return "SAFE";
    case SAFETY_DEGRADED:          return "DEGRADED";
    case SAFETY_RETURN_REQUIRED:   return "RETURN_REQUIRED";
    case SAFETY_RECOVERY_REQUIRED: return "RECOVERY_REQUIRED";
    case SAFETY_EMERGENCY:         return "EMERGENCY";
    default:                       return "?";
    }
}
