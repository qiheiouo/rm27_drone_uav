#include "guidance.h"

void guidance_takeoff(const Vec3f *target_pos, const NavState *nav, GuidanceOutput *out)
{
    out->pos_sp = *target_pos;
    out->vel_sp = vec3_zero();
    out->yaw_sp = nav->yaw;
    out->use_pos_sp = 1u;
}

void guidance_hold(const Vec3f *hold_pos, const NavState *nav, GuidanceOutput *out)
{
    out->pos_sp = *hold_pos;
    out->vel_sp = vec3_zero();
    out->yaw_sp = nav->yaw;
    out->use_pos_sp = 1u;
}

void cruise_guidance_update(const Waypoint *wp, const NavState *nav, GuidanceOutput *out)
{
    Vec3f to_wp = vec3_sub(wp->pos, nav->pos);
    float dist = vec3_norm(to_wp);

    Vec3f vel = vec3_zero();
    if (dist > 1e-4f) {
        /* 接近航点时线性减速，避免过冲 */
        float speed = wp->speed * clampf(dist / 0.5f, 0.15f, 1.0f);
        vel = vec3_scale(to_wp, speed / dist);
    }
    out->pos_sp = wp->pos;
    out->vel_sp = vel;
    out->use_pos_sp = 0u;
    if (vec3_norm_xy(to_wp) > 0.05f) {
        out->yaw_sp = atan2f(to_wp.y, to_wp.x);
    } else {
        out->yaw_sp = nav->yaw;
    }
}

void target_guidance_update(const TargetTrack *target, const NavState *nav,
                            const TerminalParams *params, GuidanceOutput *out)
{
    out->pos_sp = nav->pos;
    out->use_pos_sp = 0u;

    if (target->visible) {
        Vec3f rel = target->rel_pos;
        float range = vec3_norm(rel);
        Vec3f vel = vec3_zero();
        if (range > 1e-4f) {
            float speed = params->kp * range;
            speed = clampf(speed, params->min_closing_speed, params->approach_speed);
            vel = vec3_scale(rel, speed / range);
        }
        out->vel_sp = vel;
        if (vec3_norm_xy(rel) > 0.02f) {
            out->yaw_sp = atan2f(rel.y, rel.x);
        } else {
            out->yaw_sp = nav->yaw;
        }
    } else {
        /* 目标暂时丢失：原地减速等待，FSM 决定何时回 SEARCH */
        out->vel_sp = vec3_zero();
        out->yaw_sp = nav->yaw;
    }
}

void recovery_guidance_update(const NavState *nav, float damping_gain, GuidanceOutput *out)
{
    out->pos_sp = nav->pos;
    out->use_pos_sp = 0u;
    /* 速度阻尼：把残余速度压到 0 */
    out->vel_sp = vec3_scale(nav->vel, -damping_gain);
    out->yaw_sp = nav->yaw;
}

void home_guidance_update(const HomeTrack *home, const NavState *nav,
                          const Vec3f *home_est, const HomeParams *params,
                          GuidanceOutput *out)
{
    if (home->visible) {
        /* 视觉伺服：先横向对准，再垂直下降 */
        float vx = clampf(params->kp_lateral * home->rel_pos.x,
                          -params->max_lateral_speed, params->max_lateral_speed);
        float vy = clampf(params->kp_lateral * home->rel_pos.y,
                          -params->max_lateral_speed, params->max_lateral_speed);
        float vz = 0.0f;
        if (vec3_norm_xy(home->rel_pos) <= params->lateral_tol) {
            vz = -params->descend_speed;
        }
        out->pos_sp = nav->pos;
        out->vel_sp = vec3(vx, vy, vz);
        out->yaw_sp = nav->yaw;
        out->use_pos_sp = 0u;
    } else if (nav->pos.z <= params->blind_land_alt &&
               home->time_since_update <= params->blind_land_timeout) {
        /* 低高度短暂丢失：垂直慢降完成最后几十厘米（基座机械捕获兜底） */
        out->pos_sp = nav->pos;
        out->vel_sp = vec3(0.0f, 0.0f, -params->descend_speed);
        out->yaw_sp = nav->yaw;
        out->use_pos_sp = 0u;
    } else {
        /* 螺旋搜索：VIO 漂移导致估计 home 与真实 home 存在偏差时，
         * 以估计 home 为中心扩张螺旋，直到 marker 进入下视相机 */
        float ts = home->time_since_update;
        float radius = 0.3f + params->spiral_rate * ts;
        if (radius > params->spiral_max_radius) {
            radius = params->spiral_max_radius;
        }
        if (ts > 600.0f) { ts = 0.0f; }   /* 从未见过时从中心开始 */
        float ang = params->spiral_omega * ts;
        out->pos_sp = vec3(home_est->x + radius * cosf(ang),
                           home_est->y + radius * sinf(ang),
                           params->search_alt);
        out->vel_sp = vec3_zero();
        out->yaw_sp = nav->yaw;
        out->use_pos_sp = 1u;
    }
}

void guidance_land(const NavState *nav, float descend_speed, GuidanceOutput *out)
{
    out->pos_sp = nav->pos;
    out->vel_sp = vec3(0.0f, 0.0f, -descend_speed);
    out->yaw_sp = nav->yaw;
    out->use_pos_sp = 0u;
}
