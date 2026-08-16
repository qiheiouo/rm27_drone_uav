/*
 * nav_tasks.c - STM32 固件任务装配（参考实现）
 *
 * 与 platform/host/main.c 保持相同的模块调用顺序：
 *   传感 → 估计 → 撞击检测 → 相机重建 → trackers → 重定位
 *   → safety → FSM → 制导 → 控制 → 下发飞控
 *
 * 差异：传感器来自 nav_platform 板级接口而非仿真；
 * 撞击物理事件由 IMU 检测自然获得（无需仿真注入）。
 */
#include "nav_tasks.h"
#include "nav_platform.h"
#include "guidance.h"

void nav_app_init(NavApp *app, const NavAppConfig *cfg)
{
    app->cfg = *cfg;
    estimator_init(&app->estimator, &cfg->estimator);
    impact_detector_init(&app->impact_det, &cfg->impact);
    target_tracker_init(&app->tracker, 5.0f, 0.01f);
    home_detector_init(&app->home_det, 5.0f, 0.01f);
    safety_init(&app->safety, &cfg->safety);
    mission_fsm_init(&app->fsm, &cfg->mission, cfg->home_pos,
                     &cfg->outbound_route, &cfg->search_route);
    vf_init(&app->vf, &cfg->flow, &cfg->cam_down);
    app->traj.count = 0u;
    app->traj.duration = 0.0f;
    app->traj_t = 0.0f;
    app->traj_active = 0u;
    app->prev_state = MS_BOOT;
    app->last_impact_t = -100.0f;
    app->last_relocalize_t = -100.0f;
    app->mout.state = MS_BOOT;
    app->mout.guidance = GM_NONE;
    app->mout.current_waypoint.pos = vec3_zero();
    app->mout.current_waypoint.speed = 0.0f;
    app->mout.hold_pos = vec3_zero();
    app->mout.home_search_alt = 0.8f;
    app->mout.state_changed = 0u;
    app->mout.mission_complete = 0u;
    app->mout.mission_failed = 0u;
}

void nav_app_step(NavApp *app, float dt)
{
    const NavAppConfig *cfg = &app->cfg;
    uint32_t t_ms = nav_time_ms();
    float t = (float)t_ms * 0.001f;

    /* ---- 传感 ---- */
    ImuSample imu;
    if (nav_imu_read(&imu) != 0) {
        return;   /* 无 IMU 不运行导航环（IMU 是硬依赖） */
    }

    float tof = -1.0f;
    {
        float h;
        if (nav_tof_read(&h) == 0) {
            tof = h;
        }
    }

    OdomSample vo;
    vo.valid = 0u;
    vo.pos = vec3_zero();
    vo.vel = vec3_zero();
    vo.yaw = 0.0f;
    vo.yaw_rate = 0.0f;
    vo.att = app->estimator.out.att;
    vo.timestamp_ms = t_ms;
    {
        FlowFrame frame;
        if (nav_flow_read(&frame) == 0) {
            vf_update(&app->vf, &frame, imu.gyro, app->estimator.out.att,
                      tof, dt, &vo);
        } else {
            vo.valid = 0u;
        }
    }

    /* ---- 估计 / 撞击检测 ---- */
    estimator_update(&app->estimator, &imu, &vo, tof, dt);

    if (impact_detector_update(&app->impact_det, &imu)) {
        estimator_notify_impact(&app->estimator);
        app->last_impact_t = t;
    }

    /* ---- 相机检测与重建 ---- */
    PixelObs tpix, hpix;
    tpix.visible = (nav_camera_target_read(&tpix) == 0 && tpix.visible) ? 1u : 0u;
    hpix.visible = (nav_camera_home_read(&hpix) == 0 && hpix.visible) ? 1u : 0u;

    TargetObs tobs;
    tobs.visible = tpix.visible;
    tobs.timestamp_ms = t_ms;
    tobs.rel_pos = tpix.visible
        ? camera_reconstruct_nav(&cfg->cam_forward, tpix.u, tpix.v, tpix.size_px,
                                 cfg->target_size_m, app->estimator.out.att)
        : vec3_zero();

    HomeObs hobs;
    hobs.visible = hpix.visible;
    hobs.timestamp_ms = t_ms;
    hobs.rel_pos = hpix.visible
        ? camera_reconstruct_nav(&cfg->cam_down, hpix.u, hpix.v, hpix.size_px,
                                 cfg->marker_size_m, app->estimator.out.att)
        : vec3_zero();

    target_tracker_update(&app->tracker, &tobs, dt);
    home_detector_update(&app->home_det, &hobs, dt);

    /* ---- 第三层定位：marker 捕获 → 重定位（带冷却与门限） ---- */
    if (hobs.visible && t - app->last_relocalize_t >= 1.0f) {
        NavState abs_ref = app->estimator.out;
        abs_ref.pos = vec3_sub(cfg->home_pos, hobs.rel_pos);
        float correction = vec3_dist(abs_ref.pos, app->estimator.out.pos);
        if (correction >= 0.05f || app->estimator.out.status != EST_TRACKING) {
            estimator_notify_relocalized(&app->estimator, &abs_ref);
            vf_set_pose(&app->vf, abs_ref.pos, abs_ref.yaw);
            app->last_relocalize_t = t;
        }
    }

    safety_update(&app->safety, &app->estimator.out, &cfg->home_pos, dt);

    /* ---- 任务 FSM ---- */
    MissionInput min;
    min.nav = app->estimator.out;
    min.target = app->tracker.out;
    min.home = app->home_det.out;
    min.impact_detected = app->impact_det.triggered;
    min.start_command = 1u;
    min.time_exceeded = app->safety.time_exceeded;
    min.geofence_violation = app->safety.geofence_violation;
    min.dt = dt;

    mission_fsm_update(&app->fsm, &min, &app->mout);

    /* 新一轮目标接近时布防撞击检测器 */
    if (app->mout.state == MS_TARGET_TRACK && app->mout.state_changed) {
        impact_detector_reset(&app->impact_det);
    }

    /* ---- 制导 ---- */
    GuidanceOutput gout;
    switch (app->mout.guidance) {
    case GM_TAKEOFF:
        guidance_takeoff(&app->mout.hold_pos, &app->estimator.out, &gout);
        break;
    case GM_HOLD:
        guidance_hold(&app->mout.hold_pos, &app->estimator.out, &gout);
        break;
    case GM_WAYPOINT:
        if (!app->traj_active || app->mout.state_changed ||
            traj_done(&app->traj, app->traj_t)) {
            if (app->mout.state == MS_OUTBOUND || app->mout.state == MS_SEARCH) {
                const WaypointQueue *route = (app->mout.state == MS_OUTBOUND)
                    ? &app->fsm.outbound : &app->fsm.search;
                traj_build(&app->traj, app->estimator.out.pos,
                           app->estimator.out.vel, route, 1.4f);
            } else {
                traj_build_single(&app->traj, app->estimator.out.pos,
                                  app->estimator.out.vel,
                                  app->mout.current_waypoint.pos,
                                  app->mout.current_waypoint.speed > 0.1f ?
                                      app->mout.current_waypoint.speed :
                                      cfg->mission.cruise_speed_mps,
                                  1.4f);
            }
            app->traj_t = 0.0f;
            app->traj_active = 1u;
        }
        if (app->traj.count > 0u) {
            trajectory_guidance_update(&app->traj, app->traj_t,
                                       &app->estimator.out, &gout);
            app->traj_t += dt;
        } else {
            cruise_guidance_update(&app->mout.current_waypoint,
                                   &app->estimator.out, &gout);
        }
        break;
    case GM_TERMINAL:
        target_guidance_update(&app->tracker.out, &app->estimator.out,
                               &cfg->terminal, &gout);
        break;
    case GM_RECOVERY:
        recovery_guidance_update(&app->estimator.out, cfg->recovery_damping, &gout);
        break;
    case GM_HOME_SERVO:
        home_guidance_update(&app->home_det.out, &app->estimator.out,
                             &cfg->home_pos, &cfg->home_guidance, &gout);
        break;
    case GM_LAND:
        guidance_land(&app->estimator.out, 0.3f, &gout);
        break;
    case GM_NONE:
    default:
        gout.pos_sp = app->estimator.out.pos;
        gout.vel_sp = vec3_zero();
        gout.yaw_sp = app->estimator.out.yaw;
        gout.use_pos_sp = 0u;
        break;
    }

    /* ---- 控制与下发 ---- */
    CtrlOutput ctrl;
    pos_controller_update(&cfg->ctrl, &gout, &app->estimator.out, &ctrl);

    uint8_t armed = (app->mout.state != MS_BOOT &&
                     app->mout.state != MS_SELF_CHECK &&
                     app->mout.state != MS_DOCKED) ? 1u : 0u;
    nav_fcu_set_armed(armed);
    nav_fcu_send(&ctrl);
}
