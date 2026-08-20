#include "mission_fsm.h"

static void enter_state(MissionFsm *fsm, MissionState state,
                        MissionTransitionReason reason, MissionOutput *output)
{
    if (fsm->state != state) {
        fsm->state = state;
        fsm->state_time = 0.0f;
        fsm->transition_reason = reason;
        output->state_changed = 1u;
    }
}

static uint8_t estimator_navigation_ok(const NavState *nav)
{
    return (nav->status == EST_TRACKING || nav->status == EST_RELOCALIZED ||
            nav->status == EST_VISUAL_AIDED) ? 1u : 0u;
}

static float home_lateral_error(const HomeTrack *home)
{
    return vec3_norm_xy(home->rel_pos);
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
    fsm->return_stage = RETURN_TRANSIT;
    fsm->homing_stage = HOMING_COARSE;
    fsm->docking_stage = DOCK_SEARCH_MARKER;
    fsm->transition_reason = MISSION_REASON_NONE;
    fsm->state_time = 0.0f;
    fsm->target_visible_time = 0.0f;
    fsm->target_lost_time = 0.0f;
    fsm->estimator_lost_time = 0.0f;
    fsm->dock_contact_time = 0.0f;
    fsm->breakaway_speed = cfg->cruise_speed_mps;
    fsm->outbound = *outbound;
    fsm->search = *search;
    fsm->breakaway_target = home_pos;
    fsm->recovery_managed = 0u;
    fsm->mission_complete = 0u;
    fsm->mission_failed = 0u;
}

static void update_homing_substate(MissionFsm *fsm, const MissionInput *input)
{
    const MissionConfig *cfg = &fsm->cfg;
    float lateral = home_lateral_error(&input->home);
    switch (fsm->homing_stage) {
    case HOMING_COARSE:
        if (lateral <= cfg->homing_fine_radius_m) {
            fsm->homing_stage = HOMING_FINE_ALIGNMENT;
        }
        break;
    case HOMING_FINE_ALIGNMENT:
        if (lateral <= cfg->dock_lateral_tol_m &&
            fabsf(input->home.relative_yaw) <= cfg->homing_yaw_tol_rad) {
            fsm->homing_stage = HOMING_DESCENT;
        } else if (lateral > 1.5f * cfg->homing_fine_radius_m) {
            fsm->homing_stage = HOMING_COARSE;
        }
        break;
    case HOMING_DESCENT:
        if (input->nav.pos.z <= cfg->dock_blind_land_alt_m) {
            fsm->homing_stage = HOMING_FINAL_APPROACH;
        }
        break;
    case HOMING_FINAL_APPROACH:
    default:
        break;
    }
}

static void update_docking_substate(MissionFsm *fsm, const MissionInput *input)
{
    const MissionConfig *cfg = &fsm->cfg;
    float lateral = home_lateral_error(&input->home);
    uint8_t aligned = (input->home.visible && lateral <= cfg->dock_lateral_tol_m &&
                       fabsf(input->home.relative_yaw) <= cfg->homing_yaw_tol_rad) ? 1u : 0u;
    uint8_t capture_ok = (input->home.time_since_update < 1.0f &&
                          lateral <= cfg->dock_capture_tol_m &&
                          input->nav.pos.z <= cfg->dock_capture_max_alt_m &&
                          vec3_norm(input->nav.vel) <= cfg->dock_land_vel_max) ? 1u : 0u;

    switch (fsm->docking_stage) {
    case DOCK_SEARCH_MARKER:
        if (input->home.visible) fsm->docking_stage = DOCK_APPROACH;
        break;
    case DOCK_APPROACH:
        if (aligned) fsm->docking_stage = DOCK_ALIGN;
        else if (!input->home.visible) fsm->docking_stage = DOCK_SEARCH_MARKER;
        break;
    case DOCK_ALIGN:
        if (aligned) fsm->docking_stage = DOCK_DESCEND;
        else fsm->docking_stage = DOCK_APPROACH;
        break;
    case DOCK_DESCEND:
        if (capture_ok || input->nav.pos.z <= cfg->dock_alt_m) {
            fsm->docking_stage = DOCK_FINAL_CAPTURE;
        } else if (!input->home.visible &&
                   input->nav.pos.z > cfg->dock_blind_land_alt_m) {
            fsm->docking_stage = DOCK_SEARCH_MARKER;
        }
        break;
    case DOCK_FINAL_CAPTURE:
        if (input->dock_contact || capture_ok) {
            fsm->docking_stage = DOCK_CONTACT;
            fsm->dock_contact_time = 0.0f;
        }
        break;
    case DOCK_CONTACT:
        if (input->dock_contact || capture_ok) {
            fsm->dock_contact_time += input->dt;
        } else {
            fsm->docking_stage = DOCK_FINAL_CAPTURE;
            fsm->dock_contact_time = 0.0f;
        }
        if (input->charging_detected || input->wireless_charge_ready ||
            fsm->dock_contact_time >= cfg->dock_contact_confirm_s) {
            fsm->docking_stage = DOCK_DOCKED;
        }
        break;
    case DOCK_DOCKED:
    default:
        break;
    }
}

void mission_fsm_update(MissionFsm *fsm, const MissionInput *input,
                        MissionOutput *output)
{
    const MissionConfig *cfg = &fsm->cfg;
    uint8_t in_recovery_flow;
    uint8_t in_emergency;

    output->state_changed = 0u;
    fsm->state_time += input->dt;
    fsm->recovery_managed = input->recovery_managed;

    if (input->target.visible) {
        fsm->target_visible_time += input->dt;
        fsm->target_lost_time = 0.0f;
    } else {
        fsm->target_visible_time = 0.0f;
        fsm->target_lost_time += input->dt;
    }

    in_recovery_flow = (fsm->state == MS_IMPACT || fsm->state == MS_RECOVERY ||
                        fsm->state == MS_BREAKAWAY) ? 1u : 0u;
    in_emergency = (fsm->state == MS_ESTIMATOR_LOST ||
                    fsm->state == MS_EMERGENCY_STABILIZE ||
                    fsm->state == MS_EMERGENCY_LAND) ? 1u : 0u;

    if (!in_emergency && !fsm->mission_complete && !fsm->mission_failed) {
        if (input->geofence_violation || input->emergency_requested || input->nav_failure) {
            enter_state(fsm, MS_EMERGENCY_STABILIZE,
                        input->nav_failure ? MISSION_REASON_NAV_FAILURE : MISSION_REASON_EMERGENCY,
                        output);
        } else if (!in_recovery_flow && input->nav.status == EST_LOST) {
            fsm->resume_state = fsm->state;
            fsm->estimator_lost_time = 0.0f;
            enter_state(fsm, MS_ESTIMATOR_LOST, MISSION_REASON_ESTIMATOR_LOST, output);
        } else if ((input->time_exceeded || input->return_required) &&
                   (fsm->state == MS_OUTBOUND || fsm->state == MS_SEARCH ||
                    fsm->state == MS_TARGET_ACQUIRE || fsm->state == MS_TARGET_TRACK ||
                    fsm->state == MS_TERMINAL)) {
            fsm->return_stage = RETURN_TRANSIT;
            enter_state(fsm, MS_RETURN_HOME, MISSION_REASON_SOFT_RETURN, output);
        }
    }

    switch (fsm->state) {
    case MS_BOOT:
        enter_state(fsm, MS_SELF_CHECK, MISSION_REASON_NOMINAL, output);
        break;
    case MS_SELF_CHECK:
        if (fsm->state_time >= cfg->self_check_s && input->nav.status != EST_LOST) {
            enter_state(fsm, MS_DOCKED, MISSION_REASON_NOMINAL, output);
        }
        break;
    case MS_DOCKED:
        if (!fsm->mission_complete && input->start_command &&
            fsm->state_time >= cfg->docked_launch_delay_s) {
            enter_state(fsm, MS_TAKEOFF, MISSION_REASON_NOMINAL, output);
        }
        break;
    case MS_TAKEOFF:
        if (input->nav.pos.z >= cfg->takeoff_alt_m - 0.05f) {
            enter_state(fsm, MS_OUTBOUND, MISSION_REASON_NOMINAL, output);
        }
        break;
    case MS_OUTBOUND:
        wq_advance_if_reached(&fsm->outbound, input->nav.pos, cfg->waypoint_tol_m);
        if (wq_done(&fsm->outbound)) {
            enter_state(fsm, MS_SEARCH, MISSION_REASON_NOMINAL, output);
        }
        break;
    case MS_SEARCH:
        wq_advance_if_reached(&fsm->search, input->nav.pos, cfg->waypoint_tol_m);
        if (wq_done(&fsm->search)) wq_reset(&fsm->search);
        if (input->target.visible) {
            enter_state(fsm, MS_TARGET_ACQUIRE, MISSION_REASON_TARGET_FOUND, output);
        }
        break;
    case MS_TARGET_ACQUIRE:
        if (fsm->target_visible_time >= cfg->target_confirm_s) {
            enter_state(fsm, MS_TARGET_TRACK, MISSION_REASON_TARGET_FOUND, output);
        } else if (fsm->target_lost_time >= cfg->target_acquire_timeout_s) {
            enter_state(fsm, MS_SEARCH, MISSION_REASON_TARGET_LOST, output);
        }
        break;
    case MS_TARGET_TRACK:
        if (fsm->target_lost_time >= cfg->target_lost_timeout_s) {
            enter_state(fsm, MS_SEARCH, MISSION_REASON_TARGET_LOST, output);
        } else if (input->target.status != TARGET_TRACK_LOST &&
                   input->target.range <= cfg->terminal_range_m) {
            enter_state(fsm, MS_TERMINAL, MISSION_REASON_NOMINAL, output);
        }
        break;
    case MS_TERMINAL:
        if (input->impact_detected) {
            enter_state(fsm, MS_IMPACT, MISSION_REASON_IMPACT, output);
        } else if (fsm->target_lost_time >= cfg->target_lost_timeout_s) {
            enter_state(fsm, MS_SEARCH, MISSION_REASON_TARGET_LOST, output);
        }
        break;
    case MS_IMPACT:
        enter_state(fsm, MS_RECOVERY, MISSION_REASON_IMPACT, output);
        break;
    case MS_RECOVERY:
        if (input->recovery_managed) {
            if (input->recovery_failed) {
                enter_state(fsm, MS_EMERGENCY_LAND, MISSION_REASON_EMERGENCY, output);
            } else if (input->recovery_ready_for_breakaway) {
                enter_state(fsm, MS_BREAKAWAY, MISSION_REASON_RECOVERY_COMPLETE, output);
            }
        } else {
            float tilt = acosf(clampf(
                quat_rotate(input->nav.att, vec3(0.0f, 0.0f, 1.0f)).z, -1.0f, 1.0f));
            if (fsm->state_time >= cfg->recovery_hold_s &&
                estimator_navigation_ok(&input->nav) && tilt <= cfg->recovery_tilt_ok_rad) {
                fsm->breakaway_target = vec3_add(input->nav.pos,
                    vec3(0.0f, 0.0f, cfg->breakaway_height_m));
                enter_state(fsm, MS_BREAKAWAY, MISSION_REASON_RECOVERY_COMPLETE, output);
            } else if (fsm->state_time >= cfg->recovery_hold_s +
                                         cfg->estimator_lost_timeout_s) {
                enter_state(fsm, MS_EMERGENCY_LAND, MISSION_REASON_EMERGENCY, output);
            }
        }
        break;
    case MS_BREAKAWAY:
        if (input->recovery_managed) {
            if (input->recovery_failed) {
                enter_state(fsm, MS_EMERGENCY_LAND, MISSION_REASON_EMERGENCY, output);
            } else if (input->recovery_complete) {
                fsm->return_stage = RETURN_TRANSIT;
                enter_state(fsm, MS_RETURN_HOME, MISSION_REASON_RECOVERY_COMPLETE, output);
            }
        } else if (vec3_dist(input->nav.pos, fsm->breakaway_target) <= cfg->waypoint_tol_m) {
            fsm->return_stage = RETURN_TRANSIT;
            enter_state(fsm, MS_RETURN_HOME, MISSION_REASON_RECOVERY_COMPLETE, output);
        }
        break;
    case MS_RETURN_HOME: {
        float distance_home = vec3_dist_xy(input->nav.pos, fsm->home_pos);
        fsm->return_stage = distance_home <= cfg->home_approach_radius_m
            ? RETURN_HOME_APPROACH : RETURN_TRANSIT;
        if (distance_home <= cfg->home_region_tol_m) {
            fsm->return_stage = RETURN_HOME_SEARCH;
            enter_state(fsm, MS_HOME_SEARCH, MISSION_REASON_NOMINAL, output);
        }
        break;
    }
    case MS_HOME_SEARCH:
        fsm->return_stage = RETURN_HOME_SEARCH;
        if (input->home.visible) {
            fsm->return_stage = RETURN_HOME_ACQUIRED;
            fsm->homing_stage = HOMING_COARSE;
            enter_state(fsm, MS_HOMING, MISSION_REASON_HOME_ACQUIRED, output);
        }
        break;
    case MS_HOMING:
        fsm->return_stage = RETURN_HOMING;
        if (!input->home.visible &&
            input->home.time_since_update >= cfg->home_lost_timeout_s &&
            input->nav.pos.z > cfg->dock_blind_land_alt_m) {
            enter_state(fsm, MS_HOME_SEARCH, MISSION_REASON_HOME_LOST, output);
        } else {
            update_homing_substate(fsm, input);
            if (fsm->homing_stage == HOMING_FINAL_APPROACH) {
                fsm->docking_stage = input->home.visible ? DOCK_APPROACH : DOCK_SEARCH_MARKER;
                enter_state(fsm, MS_DOCKING, MISSION_REASON_NOMINAL, output);
            }
        }
        break;
    case MS_DOCKING:
        update_docking_substate(fsm, input);
        if (fsm->docking_stage == DOCK_DOCKED) {
            fsm->mission_complete = 1u;
            enter_state(fsm, MS_DOCKED, MISSION_REASON_NOMINAL, output);
        } else if (fsm->state_time >= cfg->docking_timeout_s) {
            enter_state(fsm, MS_EMERGENCY_LAND, MISSION_REASON_NAV_FAILURE, output);
        } else if (fsm->docking_stage == DOCK_SEARCH_MARKER &&
                   input->nav.pos.z > cfg->dock_blind_land_alt_m &&
                   input->home.time_since_update >= cfg->home_lost_timeout_s) {
            enter_state(fsm, MS_HOME_SEARCH, MISSION_REASON_HOME_LOST, output);
        }
        break;
    case MS_ESTIMATOR_LOST:
        if (estimator_navigation_ok(&input->nav)) {
            enter_state(fsm, fsm->resume_state, MISSION_REASON_NOMINAL, output);
        } else {
            fsm->estimator_lost_time += input->dt;
            if (fsm->estimator_lost_time >= cfg->estimator_lost_timeout_s) {
                enter_state(fsm, MS_EMERGENCY_LAND, MISSION_REASON_EMERGENCY, output);
            }
        }
        break;
    case MS_EMERGENCY_STABILIZE:
        if (fsm->state_time >= 1.0f) {
            enter_state(fsm, MS_EMERGENCY_LAND, MISSION_REASON_EMERGENCY, output);
        }
        break;
    case MS_EMERGENCY_LAND:
        if (input->nav.pos.z <= 0.05f && !fsm->mission_complete) {
            fsm->mission_failed = 1u;
        }
        break;
    default:
        enter_state(fsm, MS_EMERGENCY_LAND, MISSION_REASON_NAV_FAILURE, output);
        break;
    }

    output->state = fsm->state;
    output->mission_complete = fsm->mission_complete;
    output->mission_failed = fsm->mission_failed;
    output->guidance = GM_NONE;
    output->home_search_alt = cfg->home_search_alt_m;
    output->return_stage = fsm->return_stage;
    output->homing_stage = fsm->homing_stage;
    output->docking_stage = fsm->docking_stage;
    output->transition_reason = fsm->transition_reason;

    switch (fsm->state) {
    case MS_TAKEOFF:
        output->guidance = GM_TAKEOFF;
        output->hold_pos = vec3(fsm->home_pos.x, fsm->home_pos.y, cfg->takeoff_alt_m);
        break;
    case MS_OUTBOUND:
    case MS_SEARCH: {
        const WaypointQueue *queue = (fsm->state == MS_OUTBOUND)
            ? &fsm->outbound : &fsm->search;
        const Waypoint *waypoint = wq_current(queue);
        output->guidance = GM_WAYPOINT;
        output->current_waypoint.pos = input->nav.pos;
        output->current_waypoint.speed = 0.0f;
        if (waypoint != 0) output->current_waypoint = *waypoint;
        break;
    }
    case MS_TARGET_ACQUIRE:
        output->guidance = GM_HOLD;
        output->hold_pos = input->nav.pos;
        break;
    case MS_TARGET_TRACK:
    case MS_TERMINAL:
        output->guidance = GM_TERMINAL;
        break;
    case MS_IMPACT:
    case MS_RECOVERY:
        output->guidance = GM_RECOVERY;
        break;
    case MS_BREAKAWAY:
        if (fsm->recovery_managed) {
            output->guidance = GM_RECOVERY;
        } else {
            output->guidance = GM_WAYPOINT;
            output->current_waypoint.pos = fsm->breakaway_target;
            output->current_waypoint.speed = fsm->breakaway_speed;
        }
        break;
    case MS_RETURN_HOME:
        output->guidance = GM_WAYPOINT;
        output->current_waypoint.pos = vec3(fsm->home_pos.x, fsm->home_pos.y,
                                            cfg->takeoff_alt_m);
        output->current_waypoint.speed = cfg->cruise_speed_mps;
        break;
    case MS_HOME_SEARCH:
    case MS_HOMING:
    case MS_DOCKING:
        output->guidance = GM_HOME_SERVO;
        break;
    case MS_ESTIMATOR_LOST:
    case MS_EMERGENCY_STABILIZE:
        output->guidance = GM_HOLD;
        output->hold_pos = input->nav.pos;
        break;
    case MS_EMERGENCY_LAND:
        output->guidance = GM_LAND;
        break;
    default:
        break;
    }
}

const char *mission_state_name(MissionState state)
{
    switch (state) {
    case MS_BOOT: return "BOOT";
    case MS_SELF_CHECK: return "SELF_CHECK";
    case MS_DOCKED: return "DOCKED";
    case MS_TAKEOFF: return "TAKEOFF";
    case MS_OUTBOUND: return "OUTBOUND";
    case MS_SEARCH: return "SEARCH";
    case MS_TARGET_ACQUIRE: return "TARGET_ACQUIRE";
    case MS_TARGET_TRACK: return "TARGET_TRACK";
    case MS_TERMINAL: return "TERMINAL";
    case MS_IMPACT: return "IMPACT";
    case MS_RECOVERY: return "RECOVERY";
    case MS_BREAKAWAY: return "BREAKAWAY";
    case MS_RETURN_HOME: return "RETURN_HOME";
    case MS_HOME_SEARCH: return "HOME_SEARCH";
    case MS_HOMING: return "HOMING";
    case MS_DOCKING: return "DOCKING";
    case MS_ESTIMATOR_LOST: return "ESTIMATOR_LOST";
    case MS_EMERGENCY_STABILIZE: return "EMERGENCY_STABILIZE";
    case MS_EMERGENCY_LAND: return "EMERGENCY_LAND";
    default: return "?";
    }
}

const char *return_stage_name(ReturnStage stage)
{
    switch (stage) {
    case RETURN_TRANSIT: return "RETURN_TRANSIT";
    case RETURN_HOME_APPROACH: return "HOME_APPROACH";
    case RETURN_HOME_SEARCH: return "HOME_SEARCH";
    case RETURN_HOME_ACQUIRED: return "HOME_ACQUIRED";
    case RETURN_HOMING: return "HOMING";
    default: return "?";
    }
}

const char *homing_stage_name(HomingStage stage)
{
    switch (stage) {
    case HOMING_COARSE: return "COARSE_HOMING";
    case HOMING_FINE_ALIGNMENT: return "FINE_ALIGNMENT";
    case HOMING_DESCENT: return "DESCENT";
    case HOMING_FINAL_APPROACH: return "FINAL_APPROACH";
    default: return "?";
    }
}

const char *docking_stage_name(DockingStage stage)
{
    switch (stage) {
    case DOCK_SEARCH_MARKER: return "SEARCH_MARKER";
    case DOCK_APPROACH: return "APPROACH";
    case DOCK_ALIGN: return "ALIGN";
    case DOCK_DESCEND: return "DESCEND";
    case DOCK_FINAL_CAPTURE: return "FINAL_CAPTURE";
    case DOCK_CONTACT: return "CONTACT";
    case DOCK_DOCKED: return "DOCKED";
    default: return "?";
    }
}
