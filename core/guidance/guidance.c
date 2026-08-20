#include "guidance.h"

static void output_init(GuidanceOutput *out, const NavState *nav)
{
    out->pos_sp = nav->pos;
    out->vel_sp = vec3_zero();
    out->accel_sp = vec3_zero();
    out->yaw_sp = nav->yaw;
    out->use_pos_sp = 0u;
    out->use_accel_sp = 0u;
}

void terminal_guidance_init(TerminalGuidance *guidance, const TerminalParams *params)
{
    guidance->cfg = *params;
    terminal_guidance_reset(guidance);
}

void terminal_guidance_reset(TerminalGuidance *guidance)
{
    guidance->stage = TERMINAL_PURSUIT;
    guidance->previous_velocity = vec3_zero();
    guidance->stage_time = 0.0f;
    guidance->lost_time = 0.0f;
    guidance->contact_expected = 0u;
    guidance->failed = 0u;
}

static TerminalStage stage_from_range(const TerminalParams *cfg, float range)
{
    if (range <= cfg->contact_range_m) {
        return TERMINAL_CONTACT_APPROACH;
    }
    if (range <= cfg->final_align_range_m) {
        return TERMINAL_FINAL_ALIGN;
    }
    if (range <= cfg->closing_range_m) {
        return TERMINAL_CLOSING;
    }
    return TERMINAL_PURSUIT;
}

void terminal_guidance_update(TerminalGuidance *guidance,
                              const TargetTrack *target,
                              const NavState *nav,
                              float dt,
                              GuidanceOutput *out)
{
    const TerminalParams *cfg = &guidance->cfg;
    float step = (nav_isfinite(dt) && dt > 1e-4f && dt <= 0.2f) ? dt : 0.01f;
    uint8_t usable = (target->status != TARGET_TRACK_LOST &&
                      target->confidence >= cfg->min_confidence) ? 1u : 0u;
    Vec3f desired = vec3_zero();
    TerminalStage next_stage = guidance->stage;
    float speed_limit = cfg->approach_speed;

    output_init(out, nav);
    guidance->stage_time += step;
    guidance->contact_expected = 0u;

    if (!usable) {
        guidance->lost_time += step;
        next_stage = TERMINAL_REACQUIRE;
        if (guidance->lost_time <= cfg->loss_grace_s) {
            desired = vec3_scale(guidance->previous_velocity,
                                 clampf(1.0f - guidance->lost_time /
                                        (cfg->loss_grace_s + 1e-3f), 0.0f, 1.0f));
        } else if (guidance->lost_time >= cfg->reacquire_timeout_s) {
            next_stage = TERMINAL_FAILED;
            guidance->failed = 1u;
        }
    } else {
        Vec3f rel = target->rel_pos;
        float yaw_target = atan2f(target->bearing.y, target->bearing.x);
        float yaw_error = wrap_pi(yaw_target - nav->yaw);

        guidance->lost_time = target->time_since_update;
        next_stage = stage_from_range(cfg, target->range);
        switch (next_stage) {
        case TERMINAL_PURSUIT:
            speed_limit = cfg->approach_speed;
            break;
        case TERMINAL_CLOSING:
            speed_limit = cfg->closing_speed_mps;
            break;
        case TERMINAL_FINAL_ALIGN:
            speed_limit = cfg->final_speed_mps;
            break;
        case TERMINAL_CONTACT_APPROACH:
            speed_limit = cfg->contact_speed_mps;
            guidance->contact_expected = 1u;
            break;
        default:
            break;
        }

        desired = vec3_add(target->rel_vel, vec3(cfg->kp * rel.x,
                                                 cfg->kp * rel.y,
                                                 cfg->kp_vertical * rel.z));
        desired.z = clampf(desired.z, -cfg->max_vertical_speed_mps,
                           cfg->max_vertical_speed_mps);
        desired = vec3_clamp_norm(desired, speed_limit);

        if ((next_stage == TERMINAL_FINAL_ALIGN ||
             next_stage == TERMINAL_CONTACT_APPROACH) &&
            fabsf(yaw_error) > cfg->yaw_align_tolerance_rad) {
            desired.x *= 0.35f;
            desired.y *= 0.35f;
        }
        if (vec3_norm(desired) < cfg->min_closing_speed &&
            target->range > cfg->contact_range_m) {
            desired = vec3_scale(target->bearing, cfg->min_closing_speed);
        }
        out->yaw_sp = yaw_target;
    }

    if (next_stage != guidance->stage) {
        guidance->stage = next_stage;
        guidance->stage_time = 0.0f;
    }

    out->vel_sp = desired;
    out->accel_sp = vec3_clamp_norm(
        vec3_scale(vec3_sub(desired, guidance->previous_velocity), 1.0f / step),
        cfg->max_accel_mps2);
    out->use_accel_sp = 1u;
    guidance->previous_velocity = desired;
}

const char *terminal_stage_name(TerminalStage stage)
{
    switch (stage) {
    case TERMINAL_PURSUIT:          return "PURSUIT";
    case TERMINAL_CLOSING:          return "CLOSING";
    case TERMINAL_FINAL_ALIGN:      return "FINAL_ALIGN";
    case TERMINAL_CONTACT_APPROACH: return "CONTACT_APPROACH";
    case TERMINAL_REACQUIRE:        return "REACQUIRE";
    case TERMINAL_FAILED:           return "FAILED";
    default:                        return "?";
    }
}

void guidance_takeoff(const Vec3f *target_pos, const NavState *nav, GuidanceOutput *out)
{
    output_init(out, nav);
    out->pos_sp = *target_pos;
    out->use_pos_sp = 1u;
}

void guidance_hold(const Vec3f *hold_pos, const NavState *nav, GuidanceOutput *out)
{
    output_init(out, nav);
    out->pos_sp = *hold_pos;
    out->use_pos_sp = 1u;
}

void cruise_guidance_update(const Waypoint *wp, const NavState *nav, GuidanceOutput *out)
{
    Vec3f to_waypoint = vec3_sub(wp->pos, nav->pos);
    float distance = vec3_norm(to_waypoint);
    output_init(out, nav);
    out->pos_sp = wp->pos;
    if (distance > 1e-4f) {
        float speed = wp->speed * clampf(distance / 0.5f, 0.15f, 1.0f);
        out->vel_sp = vec3_scale(to_waypoint, speed / distance);
    }
    if (vec3_norm_xy(to_waypoint) > 0.05f) {
        out->yaw_sp = atan2f(to_waypoint.y, to_waypoint.x);
    }
}

void trajectory_guidance_update(const Trajectory *traj, float t,
                                const NavState *nav, GuidanceOutput *out)
{
    output_init(out, nav);
    traj_evaluate(traj, t, &out->pos_sp, &out->vel_sp, &out->accel_sp);
    out->use_pos_sp = 1u;
    out->use_accel_sp = 1u;
    if (vec3_norm_xy(out->vel_sp) > 0.2f) {
        out->yaw_sp = atan2f(out->vel_sp.y, out->vel_sp.x);
    }
}

void target_guidance_update(const TargetTrack *target, const NavState *nav,
                            const TerminalParams *params, GuidanceOutput *out)
{
    TerminalGuidance guidance;
    terminal_guidance_init(&guidance, params);
    terminal_guidance_update(&guidance, target, nav, 0.01f, out);
}

void recovery_guidance_update(const NavState *nav, float damping_gain, GuidanceOutput *out)
{
    float tilt = acosf(clampf(quat_rotate(nav->att, vec3(0.0f, 0.0f, 1.0f)).z,
                              -1.0f, 1.0f));
    output_init(out, nav);
    if (tilt > 0.35f) {
        out->vel_sp = vec3(0.0f, 0.0f, 0.4f);
    } else {
        out->vel_sp = vec3_scale(nav->vel, -damping_gain);
        out->vel_sp.z += 0.2f;
    }
}

void home_guidance_update(const HomeTrack *home, const NavState *nav,
                          const Vec3f *home_est, const HomeParams *params,
                          GuidanceOutput *out)
{
    output_init(out, nav);
    if (home->visible) {
        float lateral = vec3_norm_xy(home->rel_pos);
        out->vel_sp.x = clampf(params->kp_lateral * home->rel_pos.x,
                               -params->max_lateral_speed, params->max_lateral_speed);
        out->vel_sp.y = clampf(params->kp_lateral * home->rel_pos.y,
                               -params->max_lateral_speed, params->max_lateral_speed);
        if (lateral <= params->lateral_tol &&
            fabsf(home->relative_yaw) <= params->yaw_tolerance_rad) {
            out->vel_sp.z = -params->descend_speed;
        }
        out->yaw_sp = wrap_pi(nav->yaw + params->kp_yaw * home->relative_yaw);
    } else if (nav->pos.z <= params->blind_land_alt &&
               home->time_since_update <= params->blind_land_timeout) {
        out->vel_sp.z = -params->descend_speed;
    } else {
        float time_since_seen = home->time_since_update;
        float radius;
        float angle;
        if (time_since_seen > 600.0f) {
            time_since_seen = 0.0f;
        }
        radius = clampf(0.3f + params->spiral_rate * time_since_seen,
                        0.3f, params->spiral_max_radius);
        angle = params->spiral_omega * time_since_seen;
        out->pos_sp = vec3(home_est->x + radius * cosf(angle),
                           home_est->y + radius * sinf(angle),
                           params->search_alt);
        out->use_pos_sp = 1u;
    }
}

void guidance_land(const NavState *nav, float descend_speed, GuidanceOutput *out)
{
    output_init(out, nav);
    out->vel_sp.z = -descend_speed;
}
