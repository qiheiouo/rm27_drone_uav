/* STM32 task adapter matching the host-side Plan-B algorithm call chain. */
#include "nav_tasks.h"
#include "nav_platform.h"

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

void nav_app_init(NavApp *app, const NavAppConfig *cfg)
{
    app->cfg = *cfg;
    estimator_init(&app->estimator, &cfg->estimator);
    impact_detector_init(&app->impact_detector, &cfg->impact);
    impact_detector_configure_fusion(&app->impact_detector, &cfg->impact_fusion);
    impact_recovery_init(&app->recovery, &cfg->recovery);
    app->recovery_output.stage = RECOVERY_IDLE;
    app->recovery_output.hold_position = cfg->home_pos;
    app->recovery_output.desired_velocity = vec3_zero();
    app->recovery_output.breakaway_target = cfg->home_pos;
    app->recovery_output.use_position = 0u;
    app->recovery_output.ready_for_breakaway = 0u;
    app->recovery_output.complete = 0u;
    app->recovery_output.failed = 0u;
    app->recovery_output.emergency_fallback = 0u;
    target_tracker_init(&app->target_tracker, 5.0f, 0.01f);
    home_detector_init(&app->home_detector, 5.0f, 0.01f);
    safety_init(&app->safety, &cfg->safety);
    mission_fsm_init(&app->mission, &cfg->mission, cfg->home_pos,
                     &cfg->outbound_route, &cfg->search_route);
    terminal_guidance_init(&app->terminal, &cfg->terminal);
    vf_init(&app->vision_frontend, &cfg->flow, &cfg->cam_down);
    obstacle_set_init(&app->obstacles);
    swarm_view_init(&app->swarm, cfg->self_agent_id);
    app->trajectory.count = 0u;
    app->trajectory.duration = 0.0f;
    app->trajectory_time_s = 0.0f;
    app->trajectory_active = 0u;
    app->trajectory_valid = 1u;
    app->last_relocalization_ms = 0u;
    app->previous_control.accel_cmd = vec3_zero();
    app->previous_control.yaw_rate_cmd = 0.0f;
    app->mission_output.state = MS_BOOT;
    app->mission_output.guidance = GM_NONE;
    app->mission_output.state_changed = 0u;
    app->mission_output.mission_complete = 0u;
    app->mission_output.mission_failed = 0u;
}

void nav_app_step(NavApp *app, float dt)
{
    const NavAppConfig *cfg = &app->cfg;
    uint32_t time_ms = nav_time_ms();
    ImuSample imu;
    OdomSample odometry;
    FlowFrame frame;
    PixelObs target_pixel;
    PixelObs home_pixel;
    TargetObs target_observation;
    HomeObs home_observation;
    ImpactReport impact_report;
    ObstacleRiskReport obstacle_report;
    CollisionReport swarm_report;
    SafetyInput safety_input;
    SafetyDecision safety_decision;
    MissionInput mission_input;
    NavMissionCommand mission_command;
    NavDockStatus dock_status;
    GuidanceOutput guidance;
    float tof_height = -1.0f;
    uint8_t armed;

    if (nav_imu_read(&imu) != 0) {
        nav_fcu_set_armed(0u);
        return;
    }
    (void)nav_tof_read(&tof_height);

    odometry.valid = 0u;
    odometry.pos = vec3_zero();
    odometry.vel = vec3_zero();
    odometry.yaw = 0.0f;
    odometry.yaw_rate = 0.0f;
    odometry.att = app->estimator.out.att;
    odometry.timestamp_ms = time_ms;
    if (nav_flow_read(&frame) == 0) {
        vf_update(&app->vision_frontend, &frame, imu.gyro,
                  app->estimator.out.att, tof_height, dt, &odometry);
    }
    estimator_update(&app->estimator, &imu, &odometry, tof_height, dt);

    impact_detector_arm(&app->impact_detector,
        (app->mission.state == MS_TARGET_TRACK ||
         app->mission.state == MS_TERMINAL) ? 1u : 0u);
    impact_report = impact_detector_update_fused(&app->impact_detector, &imu,
                    app->estimator.out.vel, app->estimator.out.att, dt);
    if (impact_report.rising_edge) {
        estimator_notify_impact(&app->estimator);
        impact_recovery_start(&app->recovery, &app->estimator.out);
    }

    target_pixel.visible = 0u;
    home_pixel.visible = 0u;
    if (nav_camera_target_read(&target_pixel) != 0) target_pixel.visible = 0u;
    if (nav_camera_home_read(&home_pixel) != 0) home_pixel.visible = 0u;

    target_observation.visible = target_pixel.visible;
    target_observation.timestamp_ms = time_ms;
    target_observation.confidence = target_pixel.visible ? 1.0f : 0.0f;
    target_observation.rel_pos = target_pixel.visible
        ? camera_reconstruct_nav(&cfg->cam_forward, target_pixel.u,
            target_pixel.v, target_pixel.size_px, cfg->target_size_m,
            app->estimator.out.att)
        : vec3_zero();
    target_observation.bearing = vec3_normalize_or(target_observation.rel_pos,
                                                   vec3(1.0f, 0.0f, 0.0f));

    home_observation.visible = home_pixel.visible;
    home_observation.timestamp_ms = time_ms;
    home_observation.confidence = home_pixel.visible ? 1.0f : 0.0f;
    home_observation.rel_pos = home_pixel.visible
        ? camera_reconstruct_nav(&cfg->cam_down, home_pixel.u, home_pixel.v,
            home_pixel.size_px, cfg->marker_size_m, app->estimator.out.att)
        : vec3_zero();
    home_observation.relative_yaw = home_pixel.visible
        ? wrap_pi(-app->estimator.out.yaw) : 0.0f;

    target_tracker_update(&app->target_tracker, &target_observation, dt);
    home_detector_update(&app->home_detector, &home_observation, dt);

    if (home_observation.visible &&
        (uint32_t)(time_ms - app->last_relocalization_ms) >= 1000u) {
        NavState absolute_reference = app->estimator.out;
        absolute_reference.pos = vec3_sub(cfg->home_pos,
                                          home_observation.rel_pos);
        if (vec3_dist(absolute_reference.pos, app->estimator.out.pos) >= 0.05f ||
            app->estimator.out.status != EST_TRACKING) {
            estimator_notify_relocalized(&app->estimator, &absolute_reference);
            vf_set_pose(&app->vision_frontend, absolute_reference.pos,
                        absolute_reference.yaw);
            app->last_relocalization_ms = time_ms;
        }
    }

    impact_recovery_update(&app->recovery, &app->estimator.out,
                           odometry.valid, dt, &app->recovery_output);

    obstacle_set_init(&app->obstacles);
    (void)nav_local_obstacles_read(&app->obstacles);
    obstacle_report = obstacle_evaluate(&cfg->obstacle_avoidance,
        app->estimator.out.pos, app->estimator.out.vel,
        app->estimator.out.vel, &app->obstacles);

    {
        SwarmView received;
        Vec3f future_points[8];
        swarm_view_init(&received, cfg->self_agent_id);
        if (nav_swarm_view_read(&received) == 0) {
            app->swarm = received;
        } else {
            app->swarm.other_count = 0u;
        }
        app->swarm.self.agent_id = cfg->self_agent_id;
        app->swarm.self.sequence++;
        app->swarm.self.timestamp_ms = time_ms;
        app->swarm.self.pos = app->estimator.out.pos;
        app->swarm.self.vel = app->estimator.out.vel;
        app->swarm.self.quality = app->estimator.out.quality;
        app->swarm.self.valid = 1u;
        sample_future_points(app->estimator.out.pos, app->estimator.out.vel,
                             cfg->swarm_collision.sample_dt_s,
                             future_points, 8u);
        swarm_report = collision_check(&cfg->swarm_collision,
                                        future_points, 8u, &app->swarm);
        nav_swarm_state_send(&app->swarm.self);
    }

    if (swarm_report.risk == SWARM_RISK_CRITICAL) {
        obstacle_report.level = COLLISION_RISK_CRITICAL;
    } else if (swarm_report.risk == SWARM_RISK_WARNING &&
               obstacle_report.level == COLLISION_RISK_NONE) {
        obstacle_report.level = COLLISION_RISK_WARNING;
    }

    safety_input.nav = &app->estimator.out;
    safety_input.home_position = cfg->home_pos;
    safety_input.impact_state = impact_report.rising_edge
        ? CONFIRMED_IMPACT : NO_IMPACT;
    safety_input.collision_risk = obstacle_report.level;
    safety_input.trajectory_valid = app->trajectory_valid;
    safety_input.controller_saturated = controller_saturated(
        &app->previous_control, &cfg->ctrl);
    safety_input.dt = dt;
    safety_decision = safety_update_full(&app->safety, &safety_input);

    mission_command.start = 0u;
    mission_command.request_return = 0u;
    mission_command.request_emergency = 0u;
    (void)nav_mission_command_read(&mission_command);
    dock_status.contact = 0u;
    dock_status.charging = 0u;
    dock_status.wireless_charge_ready = 0u;
    (void)nav_dock_status_read(&dock_status);

    mission_input.nav = app->estimator.out;
    mission_input.target = app->target_tracker.out;
    mission_input.home = app->home_detector.out;
    mission_input.impact_detected = app->impact_detector.triggered;
    mission_input.start_command = mission_command.start;
    mission_input.time_exceeded = app->safety.time_exceeded;
    mission_input.return_required = (uint8_t)(safety_decision.request_return ||
                                               mission_command.request_return);
    mission_input.emergency_requested = (uint8_t)(
        safety_decision.request_emergency || mission_command.request_emergency);
    if (app->recovery.active &&
        (safety_decision.reason_mask &
         ~(SAFETY_REASON_ESTIMATOR | SAFETY_REASON_IMPACT)) == 0u &&
        !mission_command.request_emergency) {
        mission_input.emergency_requested = 0u;
    }
    mission_input.nav_failure = 0u;
    mission_input.geofence_violation = app->safety.geofence_violation;
    mission_input.recovery_managed =
        (app->recovery.stage != RECOVERY_IDLE) ? 1u : 0u;
    mission_input.recovery_ready_for_breakaway =
        app->recovery_output.ready_for_breakaway;
    mission_input.recovery_complete = app->recovery_output.complete;
    mission_input.recovery_failed = app->recovery_output.failed;
    mission_input.dock_contact = dock_status.contact;
    mission_input.charging_detected = dock_status.charging;
    mission_input.wireless_charge_ready = dock_status.wireless_charge_ready;
    mission_input.dt = dt;
    mission_fsm_update(&app->mission, &mission_input, &app->mission_output);

    if (app->mission_output.state_changed &&
        (app->mission_output.state == MS_TARGET_ACQUIRE ||
         app->mission_output.state == MS_TARGET_TRACK)) {
        terminal_guidance_reset(&app->terminal);
        impact_detector_reset(&app->impact_detector);
    }

    switch (app->mission_output.guidance) {
    case GM_TAKEOFF:
        guidance_takeoff(&app->mission_output.hold_pos,
                         &app->estimator.out, &guidance);
        break;
    case GM_HOLD:
        guidance_hold(&app->mission_output.hold_pos,
                      &app->estimator.out, &guidance);
        break;
    case GM_WAYPOINT:
        if (!app->trajectory_active || app->mission_output.state_changed ||
            traj_done(&app->trajectory, app->trajectory_time_s)) {
            app->trajectory_report = trajectory_plan_single(&app->trajectory,
                app->estimator.out.pos, app->estimator.out.vel,
                app->mission_output.current_waypoint.pos,
                app->mission_output.current_waypoint.speed > 0.1f
                    ? app->mission_output.current_waypoint.speed
                    : cfg->mission.cruise_speed_mps,
                &cfg->planner, &app->obstacles, TRAJ_BACKEND_OPTIMIZED);
            app->trajectory_time_s = 0.0f;
            app->trajectory_active = app->trajectory_report.valid;
            app->trajectory_valid = app->trajectory_report.valid;
            if (!app->trajectory_report.valid) {
                app->trajectory_valid = 1u;
            }
        }
        if (app->trajectory_active) {
            trajectory_guidance_update(&app->trajectory,
                app->trajectory_time_s, &app->estimator.out, &guidance);
            app->trajectory_time_s += dt;
        } else {
            cruise_guidance_update(&app->mission_output.current_waypoint,
                                   &app->estimator.out, &guidance);
        }
        if (app->mission_output.state == MS_SEARCH) {
            guidance.yaw_sp = wrap_pi((float)time_ms * 0.001f *
                                      cfg->mission.search_yaw_rate_rps);
        }
        break;
    case GM_TERMINAL:
        terminal_guidance_update(&app->terminal, &app->target_tracker.out,
                                 &app->estimator.out, dt, &guidance);
        break;
    case GM_RECOVERY:
        fill_recovery_guidance(&app->recovery_output,
                               &app->estimator.out, &guidance);
        break;
    case GM_HOME_SERVO:
        home_guidance_update(&app->home_detector.out, &app->estimator.out,
                             &cfg->home_pos, &cfg->home_guidance, &guidance);
        break;
    case GM_LAND:
        guidance_land(&app->estimator.out, 0.3f, &guidance);
        break;
    case GM_NONE:
    default:
        guidance_output_hold(&guidance, &app->estimator.out);
        guidance.use_pos_sp = 0u;
        break;
    }

    obstacle_report = obstacle_evaluate(&cfg->obstacle_avoidance,
        app->estimator.out.pos, app->estimator.out.vel,
        guidance.vel_sp, &app->obstacles);
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
                          &app->estimator.out, &app->previous_control);
    armed = (app->mission_output.state != MS_BOOT &&
             app->mission_output.state != MS_SELF_CHECK &&
             app->mission_output.state != MS_DOCKED) ? 1u : 0u;
    nav_fcu_set_armed(armed);
    nav_fcu_send(&app->previous_control);
}
