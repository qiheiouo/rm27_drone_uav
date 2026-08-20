#include "impact_recovery.h"

static void enter_stage(ImpactRecovery *recovery, RecoveryStage stage)
{
    recovery->stage = stage;
    recovery->stage_time = 0.0f;
}

void impact_recovery_default_config(ImpactRecoveryConfig *cfg)
{
    cfg->visual_hold_s = 0.25f;
    cfg->attitude_timeout_s = 1.5f;
    cfg->velocity_timeout_s = 1.5f;
    cfg->estimator_timeout_s = 2.0f;
    cfg->total_timeout_s = 8.0f;
    cfg->tilt_ok_rad = 0.30f;
    cfg->speed_ok_mps = 0.35f;
    cfg->damping_gain = 0.9f;
    cfg->climb_speed_mps = 0.55f;
    cfg->breakaway_height_m = 0.5f;
    cfg->breakaway_tolerance_m = 0.20f;
}

void impact_recovery_init(ImpactRecovery *recovery,
                          const ImpactRecoveryConfig *cfg)
{
    recovery->cfg = *cfg;
    recovery->stage = RECOVERY_IDLE;
    recovery->stage_time = 0.0f;
    recovery->total_time = 0.0f;
    recovery->hold_position = vec3_zero();
    recovery->breakaway_target = vec3_zero();
    recovery->active = 0u;
}

void impact_recovery_start(ImpactRecovery *recovery, const NavState *nav)
{
    recovery->hold_position = nav->pos;
    recovery->breakaway_target = vec3_add(nav->pos,
        vec3(0.0f, 0.0f, recovery->cfg.breakaway_height_m));
    recovery->total_time = 0.0f;
    recovery->active = 1u;
    enter_stage(recovery, RECOVERY_VISUAL_HOLD);
}

void impact_recovery_update(ImpactRecovery *recovery,
                            const NavState *nav,
                            uint8_t visual_available,
                            float dt,
                            ImpactRecoveryOutput *out)
{
    float step = (nav_isfinite(dt) && dt > 1e-4f && dt <= 0.2f) ? dt : 0.01f;
    float tilt = acosf(clampf(quat_rotate(nav->att, vec3(0.0f, 0.0f, 1.0f)).z,
                              -1.0f, 1.0f));

    out->stage = recovery->stage;
    out->hold_position = recovery->hold_position;
    out->desired_velocity = vec3_zero();
    out->breakaway_target = recovery->breakaway_target;
    out->use_position = 0u;
    out->ready_for_breakaway = 0u;
    out->complete = 0u;
    out->failed = 0u;
    out->emergency_fallback = 0u;

    if (!recovery->active) {
        return;
    }
    recovery->stage_time += step;
    recovery->total_time += step;

    if (recovery->stage != RECOVERY_BREAKAWAY &&
        recovery->total_time >= recovery->cfg.total_timeout_s) {
        enter_stage(recovery, RECOVERY_FAILED);
    }

    switch (recovery->stage) {
    case RECOVERY_VISUAL_HOLD:
        out->use_position = visual_available ? 1u : 0u;
        out->desired_velocity.z = 0.15f;
        if (recovery->stage_time >= recovery->cfg.visual_hold_s) {
            enter_stage(recovery, RECOVERY_ATTITUDE);
        }
        break;
    case RECOVERY_ATTITUDE:
        out->desired_velocity.z = recovery->cfg.climb_speed_mps;
        if (tilt <= recovery->cfg.tilt_ok_rad) {
            enter_stage(recovery, RECOVERY_VELOCITY_DAMPING);
        } else if (recovery->stage_time >= recovery->cfg.attitude_timeout_s) {
            enter_stage(recovery, RECOVERY_FAILED);
        }
        break;
    case RECOVERY_VELOCITY_DAMPING:
        out->desired_velocity = vec3_scale(nav->vel, -recovery->cfg.damping_gain);
        out->desired_velocity.z += 0.15f;
        if (vec3_norm(nav->vel) <= recovery->cfg.speed_ok_mps) {
            enter_stage(recovery, RECOVERY_ESTIMATOR_CHECK);
        } else if (recovery->stage_time >= recovery->cfg.velocity_timeout_s) {
            enter_stage(recovery, RECOVERY_ESTIMATOR_CHECK);
        }
        break;
    case RECOVERY_ESTIMATOR_CHECK:
        out->use_position = (nav->status != EST_LOST) ? 1u : 0u;
        if (nav->status == EST_TRACKING || nav->status == EST_RELOCALIZED ||
            (nav->status == EST_DEGRADED && visual_available)) {
            enter_stage(recovery, RECOVERY_BREAKAWAY);
            out->ready_for_breakaway = 1u;
        } else if (recovery->stage_time >= recovery->cfg.estimator_timeout_s) {
            enter_stage(recovery, RECOVERY_FAILED);
        }
        break;
    case RECOVERY_BREAKAWAY: {
        Vec3f error = vec3_sub(recovery->breakaway_target, nav->pos);
        out->ready_for_breakaway = 1u;
        out->use_position = 1u;
        out->hold_position = recovery->breakaway_target;
        out->desired_velocity = vec3_clamp_norm(error, recovery->cfg.climb_speed_mps);
        if (vec3_norm(error) <= recovery->cfg.breakaway_tolerance_m) {
            enter_stage(recovery, RECOVERY_COMPLETE);
        }
        break;
    }
    case RECOVERY_COMPLETE:
        recovery->active = 0u;
        out->complete = 1u;
        break;
    case RECOVERY_FAILED:
        recovery->active = 0u;
        out->failed = 1u;
        out->emergency_fallback = 1u;
        break;
    case RECOVERY_IDLE:
    default:
        break;
    }

    out->stage = recovery->stage;
    if (recovery->stage == RECOVERY_BREAKAWAY) {
        out->ready_for_breakaway = 1u;
    } else if (recovery->stage == RECOVERY_COMPLETE) {
        out->complete = 1u;
    } else if (recovery->stage == RECOVERY_FAILED) {
        out->failed = 1u;
        out->emergency_fallback = 1u;
    }
}

const char *recovery_stage_name(RecoveryStage stage)
{
    switch (stage) {
    case RECOVERY_IDLE:             return "IDLE";
    case RECOVERY_VISUAL_HOLD:      return "VISUAL_HOLD";
    case RECOVERY_ATTITUDE:         return "ATTITUDE_RECOVERY";
    case RECOVERY_VELOCITY_DAMPING: return "VELOCITY_DAMPING";
    case RECOVERY_ESTIMATOR_CHECK:  return "ESTIMATOR_CHECK";
    case RECOVERY_BREAKAWAY:        return "BREAKAWAY";
    case RECOVERY_COMPLETE:         return "COMPLETE";
    case RECOVERY_FAILED:           return "FAILED";
    default:                        return "?";
    }
}
