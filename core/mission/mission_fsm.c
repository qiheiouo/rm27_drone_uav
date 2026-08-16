#include "mission_fsm.h"

static void enter_state(MissionFsm *fsm, MissionState s, MissionOutput *out)
{
    if (fsm->state != s) {
        fsm->state = s;
        fsm->state_time = 0.0f;
        out->state_changed = 1u;
    }
}

void mission_fsm_init(MissionFsm *fsm,
                      const MissionConfig *cfg,
                      Vec3f home_pos,
                      const WaypointQueue *outbound,
                      const WaypointQueue *search)
{
    fsm->cfg = *cfg;
    fsm->home_pos = home_pos;
    fsm->state = MS_BOOT;
    fsm->resume_state = MS_BOOT;
    fsm->state_time = 0.0f;
    fsm->target_visible_time = 0.0f;
    fsm->target_lost_time = 0.0f;
    fsm->estimator_lost_time = 0.0f;
    fsm->breakaway_speed = cfg->cruise_speed_mps;
    fsm->outbound = *outbound;
    fsm->search = *search;
    fsm->breakaway_target = home_pos;
    fsm->mission_complete = 0u;
    fsm->mission_failed = 0u;
}

static uint8_t estimator_ok(const NavState *nav)
{
    return (nav->status == EST_TRACKING || nav->status == EST_RELOCALIZED) ? 1u : 0u;
}

void mission_fsm_update(MissionFsm *fsm, const MissionInput *in, MissionOutput *out)
{
    const MissionConfig *c = &fsm->cfg;

    out->state_changed = 0u;
    fsm->state_time += in->dt;

    /* 目标可见性计时 */
    if (in->target.visible) {
        fsm->target_visible_time += in->dt;
        fsm->target_lost_time = 0.0f;
    } else {
        fsm->target_visible_time = 0.0f;
        fsm->target_lost_time += in->dt;
    }

    /* ---------- 全局异常处理 ---------- */
    {
        uint8_t in_impact_flow = (fsm->state == MS_IMPACT || fsm->state == MS_RECOVERY) ? 1u : 0u;
        uint8_t in_emergency = (fsm->state == MS_ESTIMATOR_LOST ||
                                fsm->state == MS_EMERGENCY_STABILIZE ||
                                fsm->state == MS_EMERGENCY_LAND) ? 1u : 0u;
        uint8_t task_done = (fsm->mission_complete || fsm->mission_failed) ? 1u : 0u;

        if (!in_emergency && !task_done) {
            if (in->geofence_violation) {
                enter_state(fsm, MS_EMERGENCY_STABILIZE, out);
            } else if (!in_impact_flow && in->nav.status == EST_LOST) {
                /* 撞击恢复期之外的估计器丢失 → 悬停等待恢复 */
                fsm->resume_state = fsm->state;
                fsm->estimator_lost_time = 0.0f;
                enter_state(fsm, MS_ESTIMATOR_LOST, out);
            } else if (in->time_exceeded &&
                       (fsm->state == MS_OUTBOUND || fsm->state == MS_SEARCH ||
                        fsm->state == MS_TARGET_TRACK || fsm->state == MS_TERMINAL)) {
                /* 低电量/超时：放弃任务直接返航（LOW_BATTERY 行为） */
                enter_state(fsm, MS_RETURN_HOME, out);
            }
        }
    }

    /* ---------- 状态转移 ---------- */
    switch (fsm->state) {
    case MS_BOOT:
        enter_state(fsm, MS_SELF_CHECK, out);
        break;

    case MS_SELF_CHECK:
        if (fsm->state_time >= c->self_check_s) {
            enter_state(fsm, MS_DOCKED, out);
        }
        break;

    case MS_DOCKED:
        if (fsm->mission_complete) {
            break;  /* 任务完成后的终态，不再起飞 */
        }
        if (in->start_command && fsm->state_time >= c->docked_launch_delay_s) {
            enter_state(fsm, MS_TAKEOFF, out);
        }
        break;

    case MS_TAKEOFF:
        if (in->nav.pos.z >= c->takeoff_alt_m - 0.05f) {
            enter_state(fsm, MS_OUTBOUND, out);
        }
        break;

    case MS_OUTBOUND:
        wq_advance_if_reached(&fsm->outbound, in->nav.pos, c->waypoint_tol_m);
        if (wq_done(&fsm->outbound)) {
            enter_state(fsm, MS_SEARCH, out);
        }
        break;

    case MS_SEARCH:
        wq_advance_if_reached(&fsm->search, in->nav.pos, c->waypoint_tol_m);
        if (wq_done(&fsm->search)) {
            wq_reset(&fsm->search);  /* 继续扫掠，直到发现目标或超时返航 */
        }
        if (fsm->target_visible_time >= c->target_confirm_s) {
            enter_state(fsm, MS_TARGET_TRACK, out);
        }
        break;

    case MS_TARGET_TRACK:
        if (fsm->target_lost_time >= c->target_lost_timeout_s) {
            enter_state(fsm, MS_SEARCH, out);  /* TARGET_LOST 行为 */
        } else if (in->target.visible && in->target.range <= c->terminal_range_m) {
            enter_state(fsm, MS_TERMINAL, out);
        }
        break;

    case MS_TERMINAL:
        if (in->impact_detected) {
            enter_state(fsm, MS_IMPACT, out);
        } else if (fsm->target_lost_time >= c->target_lost_timeout_s) {
            enter_state(fsm, MS_SEARCH, out);
        }
        break;

    case MS_IMPACT:
        /* IMPACT 是单 tick 标记态：此期间上层应冻结/拒绝异常视觉输入 */
        enter_state(fsm, MS_RECOVERY, out);
        break;

    case MS_RECOVERY:
        /* 姿态稳定由低层飞控保证；此处做速度阻尼，
         * 退出条件：稳定时间到 + 估计器健康 + 倾角已改平 */
        {
            float tilt = acosf(clampf(
                quat_rotate(in->nav.att, vec3(0.0f, 0.0f, 1.0f)).z, -1.0f, 1.0f));
            if (fsm->state_time >= c->recovery_hold_s &&
                estimator_ok(&in->nav) && tilt <= c->recovery_tilt_ok_rad) {
                fsm->breakaway_target = vec3_add(in->nav.pos, vec3(0.0f, 0.0f, c->breakaway_height_m));
                enter_state(fsm, MS_BREAKAWAY, out);
            } else if (fsm->state_time >= c->recovery_hold_s + c->estimator_lost_timeout_s) {
                enter_state(fsm, MS_EMERGENCY_LAND, out);
            }
        }
        break;

    case MS_BREAKAWAY:
        if (vec3_dist(in->nav.pos, fsm->breakaway_target) <= c->waypoint_tol_m) {
            enter_state(fsm, MS_RETURN_HOME, out);
        }
        break;

    case MS_RETURN_HOME:
        if (vec3_dist_xy(in->nav.pos, fsm->home_pos) <= c->home_region_tol_m) {
            enter_state(fsm, MS_HOME_SEARCH, out);
        }
        break;

    case MS_HOME_SEARCH:
        if (in->home.visible) {
            enter_state(fsm, MS_DOCKING, out);
        }
        break;

    case MS_DOCKING:
        if (!in->home.visible && in->home.time_since_update >= c->home_lost_timeout_s &&
            in->nav.pos.z > c->dock_blind_land_alt_m) {
            enter_state(fsm, MS_HOME_SEARCH, out);
        } else {
            float lat = vec3_norm_xy(in->home.rel_pos);
            /* 视觉对准停靠：高度与横向均满足 */
            uint8_t visual_ok = (in->home.visible &&
                                 in->nav.pos.z <= c->dock_alt_m &&
                                 lat <= c->dock_lateral_tol_m) ? 1u : 0u;
            /* 机械捕获：已着陆（速度≈0）+ 低空 + 最后在捕获范围内。
             * 不依赖估计高度绝对精度——z 漂移时视觉判据可能永远不满足 */
            uint8_t capture_ok = (in->home.time_since_update < 1.0f &&
                                  lat <= c->dock_capture_tol_m &&
                                  in->nav.pos.z <= c->dock_capture_max_alt_m &&
                                  vec3_norm(in->nav.vel) <= c->dock_land_vel_max) ? 1u : 0u;
            if (visual_ok || capture_ok) {
                fsm->mission_complete = 1u;
                enter_state(fsm, MS_DOCKED, out);
            }
        }
        break;

    case MS_ESTIMATOR_LOST:
        if (estimator_ok(&in->nav)) {
            enter_state(fsm, fsm->resume_state, out);
        } else {
            fsm->estimator_lost_time += in->dt;
            if (fsm->estimator_lost_time >= c->estimator_lost_timeout_s) {
                enter_state(fsm, MS_EMERGENCY_LAND, out);
            }
        }
        break;

    case MS_EMERGENCY_STABILIZE:
        if (fsm->state_time >= 1.0f) {
            enter_state(fsm, MS_EMERGENCY_LAND, out);
        }
        break;

    case MS_EMERGENCY_LAND:
        if (in->nav.pos.z <= 0.05f && !fsm->mission_complete) {
            fsm->mission_failed = 1u;
        }
        break;

    default:
        enter_state(fsm, MS_EMERGENCY_LAND, out);
        break;
    }

    /* ---------- 制导模式选择 ---------- */
    out->state = fsm->state;
    out->mission_complete = fsm->mission_complete;
    out->mission_failed = fsm->mission_failed;
    out->guidance = GM_NONE;
    out->home_search_alt = c->home_search_alt_m;

    switch (fsm->state) {
    case MS_TAKEOFF:
        out->guidance = GM_TAKEOFF;
        out->hold_pos = vec3(fsm->home_pos.x, fsm->home_pos.y, c->takeoff_alt_m);
        break;

    case MS_OUTBOUND:
    case MS_SEARCH: {
        const WaypointQueue *q = (fsm->state == MS_OUTBOUND) ? &fsm->outbound : &fsm->search;
        const Waypoint *wp = wq_current(q);
        out->guidance = GM_WAYPOINT;
        if (wp != 0) {
            out->current_waypoint = *wp;
        } else {
            out->current_waypoint.pos = in->nav.pos;
            out->current_waypoint.speed = 0.0f;
        }
        break;
    }

    case MS_TARGET_TRACK:
    case MS_TERMINAL:
        out->guidance = GM_TERMINAL;
        break;

    case MS_RECOVERY:
        out->guidance = GM_RECOVERY;
        break;

    case MS_BREAKAWAY:
        out->guidance = GM_WAYPOINT;
        out->current_waypoint.pos = fsm->breakaway_target;
        out->current_waypoint.speed = fsm->breakaway_speed;
        break;

    case MS_RETURN_HOME:
        out->guidance = GM_WAYPOINT;
        out->current_waypoint.pos = vec3(fsm->home_pos.x, fsm->home_pos.y, c->takeoff_alt_m);
        out->current_waypoint.speed = c->cruise_speed_mps;
        break;

    case MS_HOME_SEARCH:
    case MS_DOCKING:
        out->guidance = GM_HOME_SERVO;
        break;

    case MS_ESTIMATOR_LOST:
    case MS_EMERGENCY_STABILIZE:
        out->guidance = GM_HOLD;
        out->hold_pos = in->nav.pos;
        break;

    case MS_EMERGENCY_LAND:
        out->guidance = GM_LAND;
        break;

    default:
        out->guidance = GM_NONE;
        break;
    }
}

const char *mission_state_name(MissionState s)
{
    switch (s) {
    case MS_BOOT:                return "BOOT";
    case MS_SELF_CHECK:          return "SELF_CHECK";
    case MS_DOCKED:              return "DOCKED";
    case MS_TAKEOFF:             return "TAKEOFF";
    case MS_OUTBOUND:            return "OUTBOUND";
    case MS_SEARCH:              return "SEARCH";
    case MS_TARGET_TRACK:        return "TARGET_TRACK";
    case MS_TERMINAL:            return "TERMINAL";
    case MS_IMPACT:              return "IMPACT";
    case MS_RECOVERY:            return "RECOVERY";
    case MS_BREAKAWAY:           return "BREAKAWAY";
    case MS_RETURN_HOME:         return "RETURN_HOME";
    case MS_HOME_SEARCH:         return "HOME_SEARCH";
    case MS_DOCKING:             return "DOCKING";
    case MS_ESTIMATOR_LOST:      return "ESTIMATOR_LOST";
    case MS_EMERGENCY_STABILIZE: return "EMERGENCY_STABILIZE";
    case MS_EMERGENCY_LAND:      return "EMERGENCY_LAND";
    default:                     return "?";
    }
}
