/* Deterministic host simulation for the real nav_core call chain. */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "scenario.h"
#include "sim_sensors.h"
#include "sim_vision.h"

static void print_scenarios(void)
{
    printf("nominal\n");
    printf("moving-target\n");
    printf("target-loss\n");
    printf("impact-degraded\n");
    printf("impact-lost\n");
    printf("home-initial-hidden\n");
    printf("home-loss\n");
    printf("local-obstacle\n");
    printf("forced-return\n");
    printf("two-agent-conflict\n");
}

static uint8_t in_window(float time, float start, float end)
{
    return (start >= 0.0f && time >= start && time <= end) ? 1u : 0u;
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

int main(int argc, char **argv)
{
    Scenario scenario;
    SimState simulation;
    SimSensors sensors;
    StateEstimator estimator;
    ImpactDetector impact_detector;
    ImpactRecovery recovery;
    ImpactRecoveryOutput recovery_output;
    TargetTracker tracker;
    HomeDetector home_detector;
    SafetyMonitor safety;
    MissionFsm mission;
    TerminalGuidance terminal;
    SwarmView swarm;
    VisionFrontend vision_frontend;
    SimVisionWorld vision_world;
    Trajectory trajectory;
    TrajectoryPlanReport trajectory_report;
    CtrlOutput control;
    float trajectory_time = 0.0f;
    uint8_t trajectory_active = 0u;
    uint8_t trajectory_valid = 1u;
    float last_impact_time = -100.0f;
    float last_relocalization_time = -100.0f;
    float time = 0.0f;
    float max_position_error = 0.0f;
    int next_log_centi = 0;
    uint8_t dock_contact = 0u;
    uint8_t target_loss_exercised = 0u;
    uint8_t recovery_degradation_exercised = 0u;
    uint8_t home_loss_exercised = 0u;
    uint8_t obstacle_avoidance_exercised = 0u;
    uint8_t trajectory_detour_exercised = 0u;
    uint8_t forced_return_exercised = 0u;
    uint8_t swarm_conflict_exercised = 0u;
    int argument_index;

    scenario_default(&scenario);
    for (argument_index = 1; argument_index < argc; argument_index++) {
        if (strcmp(argv[argument_index], "--scenario") == 0 && argument_index + 1 < argc) {
            if (scenario_apply_kind(&scenario, argv[++argument_index]) != 0) {
                printf("unknown scenario: %s\n", argv[argument_index]);
                print_scenarios();
                return 1;
            }
        } else if (strcmp(argv[argument_index], "--list-scenarios") == 0) {
            print_scenarios();
            return 0;
        } else if (strcmp(argv[argument_index], "--lost-on-impact") == 0) {
            scenario.estimator.lost_on_impact = 1u;
        } else if (strcmp(argv[argument_index], "--truth-mode") == 0) {
            scenario.estimator.mode = EST_MODE_TRUTH;
        } else if (strcmp(argv[argument_index], "--sim-vo") == 0) {
            scenario.use_flow_vo = 0u;
        } else if (strcmp(argv[argument_index], "--hard-impact") == 0) {
            scenario.impact_delta_v = vec3(-2.5f, 1.6f, 1.2f);
            scenario.impact_delta_yaw = 1.5f;
            scenario.impact_delta_pitch = 1.2f;
            scenario.impact_delta_roll = 1.4f;
            scenario.vision_freeze_s = 0.8f;
            scenario.estimator.impact_blind_s = 0.8f;
        } else if (strcmp(argv[argument_index], "--seed") == 0 && argument_index + 1 < argc) {
            scenario.seed = (uint32_t)strtoul(argv[++argument_index], 0, 10);
        } else {
            printf("unknown argument: %s\n", argv[argument_index]);
            return 1;
        }
    }

    sim_dynamics_init(&simulation, scenario.home_pos, 0.0f);
    sim_sensors_init(&sensors, scenario.seed);
    if (scenario.estimator.mode == EST_MODE_TRUTH) {
        scenario.use_flow_vo = 0u;
        sensors.vo_drift_vel = vec3_zero();
        sensors.vo_yaw_drift_rate = 0.0f;
        sensors.vo_pos_noise = 0.0f;
        sensors.vo_vel_noise = 0.0f;
        sensors.vo_yaw_noise = 0.0f;
    }
    estimator_init(&estimator, &scenario.estimator);
    impact_detector_init(&impact_detector, &scenario.impact);
    impact_detector_configure_fusion(&impact_detector, &scenario.impact_fusion);
    impact_recovery_init(&recovery, &scenario.recovery);
    recovery_output.stage = RECOVERY_IDLE;
    recovery_output.desired_velocity = vec3_zero();
    recovery_output.hold_position = scenario.home_pos;
    recovery_output.breakaway_target = scenario.home_pos;
    recovery_output.use_position = 0u;
    recovery_output.ready_for_breakaway = 0u;
    recovery_output.complete = 0u;
    recovery_output.failed = 0u;
    recovery_output.emergency_fallback = 0u;
    target_tracker_init(&tracker, 5.0f, scenario.dt);
    home_detector_init(&home_detector, 5.0f, scenario.dt);
    safety_init(&safety, &scenario.safety);
    mission_fsm_init(&mission, &scenario.mission, scenario.home_pos,
                     &scenario.outbound_route, &scenario.search_route);
    terminal_guidance_init(&terminal, &scenario.terminal);
    swarm_view_init(&swarm, 2u);
    vf_init(&vision_frontend, &scenario.flow, &scenario.cam_down);
    sim_vision_world_init(&vision_world, scenario.seed,
                          scenario.feature_area_m, scenario.feature_count);
    trajectory.count = 0u;
    trajectory.duration = 0.0f;
    trajectory_report.valid = 1u;
    control.accel_cmd = vec3_zero();
    control.yaw_rate_cmd = 0.0f;

    printf("# RM Drone Nav Plan-B algorithm prototype\n");
    printf("# scenario=%s dt=%.3f budget=%.1fs seed=%u estimator=%s\n",
           scenario.name, scenario.dt, scenario.safety.max_mission_time_s,
           scenario.seed, scenario.estimator.mode == EST_MODE_INS ? "INS+VO" : "TRUTH");

    while (time < scenario.sim_max_time_s) {
        uint32_t time_ms = (uint32_t)(time * 1000.0f);
        uint8_t impact_visual_freeze =
            (time - last_impact_time < scenario.vision_freeze_s) ? 1u : 0u;
        uint8_t target_hidden = in_window(time, scenario.target_hidden_start_s,
                                         scenario.target_hidden_end_s);
        uint8_t home_hidden =
            (scenario.home_hidden_until_s >= 0.0f && time <= scenario.home_hidden_until_s) ||
            in_window(time, scenario.home_loss_start_s, scenario.home_loss_end_s);
        ImuSample imu;
        OdomSample odometry;
        PixelObs target_pixel;
        PixelObs home_pixel;
        TargetObs target_observation;
        HomeObs home_observation;
        float tof_height;
        ImpactReport impact_report;
        ObstacleRiskReport obstacle_report;
        CollisionReport swarm_report;
        SafetyInput safety_input;
        SafetyDecision safety_decision;
        MissionInput mission_input;
        MissionOutput mission_output;
        GuidanceOutput guidance;
        SimImpact simulated_impact;

        sim_sensors_imu(&sensors, &simulation, time_ms, &imu);
        sim_sensors_target(&sensors, &simulation, &scenario.cam_forward,
                           scenario.target_pos, scenario.target_size_m,
                           (uint8_t)(impact_visual_freeze || target_hidden),
                           time_ms, &target_pixel);
        sim_sensors_home(&sensors, &simulation, &scenario.cam_down,
                         scenario.home_pos, scenario.marker_size_m,
                         (uint8_t)(impact_visual_freeze || home_hidden),
                         time_ms, &home_pixel);

        tof_height = sim_sensors_tof(&sensors, &simulation);
        if (scenario.use_flow_vo) {
            FlowFrame frame;
            sim_vision_frame(&sensors, &vision_world, &simulation,
                             &scenario.cam_down, impact_visual_freeze,
                             time_ms, &frame);
            vf_update(&vision_frontend, &frame, imu.gyro, estimator.out.att,
                      tof_height, scenario.dt, &odometry);
        } else {
            sim_sensors_vo(&sensors, &simulation, impact_visual_freeze,
                           scenario.dt, time_ms, &odometry);
        }

        estimator_update(&estimator, &imu, &odometry, tof_height, scenario.dt);
        impact_detector_arm(&impact_detector,
            (mission.state == MS_TARGET_TRACK || mission.state == MS_TERMINAL) ? 1u : 0u);
        impact_report = impact_detector_update_fused(&impact_detector, &imu,
                            estimator.out.vel, estimator.out.att, scenario.dt);
        if (impact_report.rising_edge) {
            estimator_notify_impact(&estimator);
            impact_recovery_start(&recovery, &estimator.out);
            printf("[t=%6.2f] EVENT impact confirmed confidence=%.2f\n",
                   time, impact_report.confidence);
        }

        target_observation.visible = target_pixel.visible;
        target_observation.timestamp_ms = time_ms;
        target_observation.confidence = target_pixel.visible ? 0.95f : 0.0f;
        target_observation.rel_pos = target_pixel.visible
            ? camera_reconstruct_nav(&scenario.cam_forward, target_pixel.u,
                                     target_pixel.v, target_pixel.size_px,
                                     scenario.target_size_m, estimator.out.att)
            : vec3_zero();
        target_observation.bearing = vec3_normalize_or(target_observation.rel_pos,
                                                       vec3(1.0f, 0.0f, 0.0f));

        home_observation.visible = home_pixel.visible;
        home_observation.timestamp_ms = time_ms;
        home_observation.confidence = home_pixel.visible ? 0.98f : 0.0f;
        home_observation.rel_pos = home_pixel.visible
            ? camera_reconstruct_nav(&scenario.cam_down, home_pixel.u,
                                     home_pixel.v, home_pixel.size_px,
                                     scenario.marker_size_m, estimator.out.att)
            : vec3_zero();
        home_observation.relative_yaw = home_pixel.visible ? wrap_pi(-estimator.out.yaw) : 0.0f;

        target_tracker_update(&tracker, &target_observation, scenario.dt);
        home_detector_update(&home_detector, &home_observation, scenario.dt);

        if (target_hidden && (mission.state == MS_TARGET_TRACK ||
                              mission.state == MS_TERMINAL)) {
            if (!target_loss_exercised) {
                printf("[t=%6.2f] EVENT target observation interrupted\n", time);
            }
            target_loss_exercised = 1u;
        }
        if (home_hidden && (mission.state == MS_HOME_SEARCH ||
                            mission.state == MS_HOMING ||
                            mission.state == MS_DOCKING)) {
            if (!home_loss_exercised) {
                printf("[t=%6.2f] EVENT home marker unavailable\n", time);
            }
            home_loss_exercised = 1u;
        }
        if (recovery.active && (estimator.out.status == EST_DEGRADED ||
                                estimator.out.status == EST_LOST ||
                                estimator.out.status == EST_RECOVERING)) {
            recovery_degradation_exercised = 1u;
        }

        if (home_observation.visible && time - last_relocalization_time >= 1.0f) {
            NavState absolute_reference = estimator.out;
            absolute_reference.pos = vec3_sub(scenario.home_pos, home_observation.rel_pos);
            if (vec3_dist(absolute_reference.pos, estimator.out.pos) >= 0.05f ||
                estimator.out.status != EST_TRACKING) {
                estimator_notify_relocalized(&estimator, &absolute_reference);
                vf_set_pose(&vision_frontend, absolute_reference.pos,
                            absolute_reference.yaw);
                last_relocalization_time = time;
            }
        }

        if (recovery.stage != RECOVERY_IDLE && recovery.stage != RECOVERY_COMPLETE &&
            recovery.stage != RECOVERY_FAILED) {
            impact_recovery_update(&recovery, &estimator.out,
                                   odometry.valid, scenario.dt, &recovery_output);
        } else if (recovery.stage == RECOVERY_COMPLETE || recovery.stage == RECOVERY_FAILED) {
            impact_recovery_update(&recovery, &estimator.out,
                                   odometry.valid, scenario.dt, &recovery_output);
        }

        obstacle_report = obstacle_evaluate(&scenario.obstacle_avoidance,
                            estimator.out.pos, estimator.out.vel,
                            estimator.out.vel, &scenario.obstacles);
        if (obstacle_report.level != COLLISION_RISK_NONE) {
            if (!obstacle_avoidance_exercised) {
                printf("[t=%6.2f] EVENT local obstacle risk id=%u clearance=%.2f m\n",
                       time, obstacle_report.obstacle_id,
                       obstacle_report.minimum_separation_m);
            }
            obstacle_avoidance_exercised = 1u;
        }

        swarm.self.pos = estimator.out.pos;
        swarm.self.vel = estimator.out.vel;
        swarm.self.timestamp_ms = time_ms;
        swarm.self.sequence++;
        if (scenario.other_agent_enabled) {
            Vec3f future_points[8];
            scenario.other_agent.age_s = 0.0f;
            scenario.other_agent.timestamp_ms = time_ms;
            swarm.others[0] = scenario.other_agent;
            swarm.other_count = 1u;
            sample_future_points(estimator.out.pos, estimator.out.vel,
                                 scenario.swarm_collision.sample_dt_s,
                                 future_points, 8u);
            swarm_report = collision_check(&scenario.swarm_collision,
                                            future_points, 8u, &swarm);
        } else {
            swarm.other_count = 0u;
            swarm_report = collision_check(&scenario.swarm_collision, 0, 0u, &swarm);
        }
        if (swarm_report.conflict) {
            if (!swarm_conflict_exercised) {
                printf("[t=%6.2f] EVENT swarm conflict agent=%u separation=%.2f m\n",
                       time, swarm_report.other_agent_id,
                       swarm_report.min_separation);
            }
            swarm_conflict_exercised = 1u;
        }

        safety_input.nav = &estimator.out;
        safety_input.home_position = scenario.home_pos;
        safety_input.impact_state = impact_report.rising_edge
            ? CONFIRMED_IMPACT : NO_IMPACT;
        safety_input.collision_risk = obstacle_report.level;
        safety_input.trajectory_valid = trajectory_valid;
        safety_input.controller_saturated = controller_saturated(&control, &scenario.ctrl);
        safety_input.dt = scenario.dt;
        safety_decision = safety_update_full(&safety, &safety_input);

        dock_contact = (vec3_dist_xy(simulation.pos, scenario.home_pos) <= 0.12f &&
                        simulation.pos.z <= 0.06f && vec3_norm(simulation.vel) <= 0.15f) ? 1u : 0u;
        mission_input.nav = estimator.out;
        mission_input.target = tracker.out;
        mission_input.home = home_detector.out;
        mission_input.impact_detected = impact_detector.triggered;
        mission_input.start_command = 1u;
        mission_input.time_exceeded = safety.time_exceeded;
        mission_input.return_required = safety_decision.request_return;
        mission_input.emergency_requested = safety_decision.request_emergency;
        /* A temporary estimator-only alarm is handled by the bounded impact
         * recovery state machine. Hard deadlines and geofence alarms remain
         * authoritative even during recovery. */
        if (recovery.active &&
            (safety_decision.reason_mask &
             ~(SAFETY_REASON_ESTIMATOR | SAFETY_REASON_IMPACT)) == 0u) {
            mission_input.emergency_requested = 0u;
        }
        mission_input.nav_failure = 0u;
        mission_input.geofence_violation = safety.geofence_violation;
        mission_input.recovery_managed = (recovery.stage != RECOVERY_IDLE) ? 1u : 0u;
        mission_input.recovery_ready_for_breakaway = recovery_output.ready_for_breakaway;
        mission_input.recovery_complete = recovery_output.complete;
        mission_input.recovery_failed = recovery_output.failed;
        mission_input.dock_contact = dock_contact;
        mission_input.charging_detected =
            (dock_contact && mission.docking_stage == DOCK_CONTACT) ? 1u : 0u;
        mission_input.wireless_charge_ready = dock_contact;
        mission_input.dt = scenario.dt;
        mission_fsm_update(&mission, &mission_input, &mission_output);

        if (mission_output.state_changed) {
            printf("[t=%6.2f] STATE -> %s est=%s safety=%s\n",
                   time, mission_state_name(mission_output.state),
                   est_status_name(estimator.out.status),
                   safety_level_name(safety_decision.level));
            if (mission_output.state == MS_TARGET_ACQUIRE ||
                mission_output.state == MS_TARGET_TRACK) {
                terminal_guidance_reset(&terminal);
                impact_detector_reset(&impact_detector);
            }
            if (mission_output.state == MS_RETURN_HOME) {
                forced_return_exercised = 1u;
            }
        }

        switch (mission_output.guidance) {
        case GM_TAKEOFF:
            guidance_takeoff(&mission_output.hold_pos, &estimator.out, &guidance);
            break;
        case GM_HOLD:
            guidance_hold(&mission_output.hold_pos, &estimator.out, &guidance);
            break;
        case GM_WAYPOINT:
            if (!trajectory_active || mission_output.state_changed ||
                traj_done(&trajectory, trajectory_time)) {
                /* Plan one queue edge at a time so the FSM's fixed waypoint
                 * index and the active polynomial cannot diverge after a
                 * local detour. */
                trajectory_report = trajectory_plan_single(&trajectory,
                    estimator.out.pos, estimator.out.vel,
                    mission_output.current_waypoint.pos,
                    mission_output.current_waypoint.speed > 0.1f
                        ? mission_output.current_waypoint.speed
                        : scenario.mission.cruise_speed_mps,
                    &scenario.planner, &scenario.obstacles,
                    TRAJ_BACKEND_OPTIMIZED);
                trajectory_time = 0.0f;
                trajectory_active = trajectory_report.valid;
                trajectory_valid = trajectory_report.valid;
                if (trajectory_report.detour_used) {
                    if (!trajectory_detour_exercised) {
                        printf("[t=%6.2f] EVENT fixed-memory trajectory detour selected\n",
                               time);
                    }
                    trajectory_detour_exercised = 1u;
                }
                if (!trajectory_report.valid) {
                    /* The always-available simple waypoint law is the final fallback. */
                    trajectory_valid = 1u;
                }
            }
            if (trajectory_active) {
                trajectory_guidance_update(&trajectory, trajectory_time,
                                           &estimator.out, &guidance);
                trajectory_time += scenario.dt;
            } else {
                cruise_guidance_update(&mission_output.current_waypoint,
                                       &estimator.out, &guidance);
            }
            if (mission_output.state == MS_SEARCH) {
                guidance.yaw_sp = wrap_pi(time *
                    scenario.mission.search_yaw_rate_rps);
            }
            break;
        case GM_TERMINAL:
            terminal_guidance_update(&terminal, &tracker.out, &estimator.out,
                                     scenario.dt, &guidance);
            break;
        case GM_RECOVERY:
            if (recovery.stage != RECOVERY_IDLE) {
                fill_recovery_guidance(&recovery_output, &estimator.out, &guidance);
            } else {
                recovery_guidance_update(&estimator.out,
                                         scenario.recovery.damping_gain, &guidance);
            }
            break;
        case GM_HOME_SERVO:
            home_guidance_update(&home_detector.out, &estimator.out,
                                 &scenario.home_pos, &scenario.home_guidance,
                                 &guidance);
            break;
        case GM_LAND:
            guidance_land(&estimator.out, 0.3f, &guidance);
            break;
        case GM_NONE:
        default:
            guidance_output_hold(&guidance, &estimator.out);
            guidance.use_pos_sp = 0u;
            break;
        }

        obstacle_report = obstacle_evaluate(&scenario.obstacle_avoidance,
                            estimator.out.pos, estimator.out.vel,
                            guidance.vel_sp, &scenario.obstacles);
        guidance.vel_sp = obstacle_apply_avoidance(guidance.vel_sp,
                            &obstacle_report, scenario.ctrl.max_vel);

        if (swarm_report.conflict) {
            SwarmAvoidanceDecision decision = swarm_avoidance_decide(
                &scenario.swarm_avoidance, swarm.self.agent_id,
                &swarm_report, guidance.vel_sp);
            guidance.vel_sp = vec3_clamp_norm(vec3_add(guidance.vel_sp,
                                                       decision.velocity_bias),
                                              scenario.ctrl.max_vel);
        }

        pos_controller_update(&scenario.ctrl, &guidance, &estimator.out, &control);

        simulated_impact.active = 0u;
        simulated_impact.delta_v = vec3_zero();
        simulated_impact.delta_yaw = 0.0f;
        simulated_impact.delta_pitch = 0.0f;
        simulated_impact.delta_roll = 0.0f;
        if (mission_output.state == MS_TERMINAL &&
            time - last_impact_time >= 0.5f &&
            vec3_dist(simulation.pos, scenario.target_pos) <= scenario.impact_range_m) {
            simulated_impact.active = 1u;
            simulated_impact.delta_v = scenario.impact_delta_v;
            simulated_impact.delta_yaw = scenario.impact_delta_yaw;
            simulated_impact.delta_pitch = scenario.impact_delta_pitch;
            simulated_impact.delta_roll = scenario.impact_delta_roll;
            last_impact_time = time;
            printf("[t=%6.2f] EVENT physical contact with target\n", time);
        }
        sim_dynamics_step(&simulation, &control, &scenario.dynamics,
                          &simulated_impact, scenario.dt);
        scenario.target_pos = vec3_add(scenario.target_pos,
                                       vec3_scale(scenario.target_velocity, scenario.dt));
        if (scenario.other_agent_enabled) {
            scenario.other_agent.pos = vec3_add(scenario.other_agent.pos,
                vec3_scale(scenario.other_agent.vel, scenario.dt));
        }

        if (estimator.out.status != EST_LOST) {
            float error = vec3_dist(estimator.out.pos, simulation.pos);
            if (error > max_position_error) max_position_error = error;
        }

        if ((int)(time * 100.0f) >= next_log_centi) {
            next_log_centi += 100;
            printf("[t=%6.2f] %-16s est=%-12s safety=%-17s pos=(%5.2f,%5.2f,%4.2f) "
                   "target=%-8s term=%-16s recovery=%-18s home=%c dock=%s\n",
                   time, mission_state_name(mission_output.state),
                   est_status_name(estimator.out.status),
                   safety_level_name(safety_decision.level),
                   estimator.out.pos.x, estimator.out.pos.y, estimator.out.pos.z,
                   target_track_status_name(tracker.out.status),
                   terminal_stage_name(terminal.stage),
                   recovery_stage_name(recovery.stage),
                   home_detector.out.visible ? 'Y' : 'N',
                   docking_stage_name(mission.docking_stage));
        }

        if (mission_output.mission_complete) {
            uint8_t expectations_met = 1u;
            if (scenario.kind == SCENARIO_TARGET_LOSS && !target_loss_exercised)
                expectations_met = 0u;
            if ((scenario.kind == SCENARIO_IMPACT_DEGRADED ||
                 scenario.kind == SCENARIO_IMPACT_LOST) &&
                !recovery_degradation_exercised)
                expectations_met = 0u;
            if ((scenario.kind == SCENARIO_HOME_INITIAL_HIDDEN ||
                 scenario.kind == SCENARIO_HOME_LOSS) && !home_loss_exercised)
                expectations_met = 0u;
            if (scenario.kind == SCENARIO_LOCAL_OBSTACLE &&
                (!obstacle_avoidance_exercised || !trajectory_detour_exercised))
                expectations_met = 0u;
            if (scenario.kind == SCENARIO_FORCED_RETURN && !forced_return_exercised)
                expectations_met = 0u;
            if (scenario.kind == SCENARIO_TWO_AGENT_CONFLICT && !swarm_conflict_exercised)
                expectations_met = 0u;
            if (!expectations_met) {
                printf("[t=%6.2f] MISSION_FAILED scenario=%s expected branch not exercised\n",
                       time, scenario.name);
                return 4;
            }
            printf("[t=%6.2f] MISSION_SUCCESS scenario=%s dist_to_home=%.3f m "
                   "elapsed=%.2f s remaining=%.2f s max_est_err=%.2f m\n",
                   time, scenario.name, vec3_dist(simulation.pos, scenario.home_pos),
                   time, safety.remaining_s, max_position_error);
            return 0;
        }
        if (mission_output.mission_failed) {
            printf("[t=%6.2f] MISSION_FAILED scenario=%s emergency landed "
                   "max_est_err=%.2f m\n",
                   time, scenario.name, max_position_error);
            return 2;
        }
        time += scenario.dt;
    }

    printf("MISSION_FAILED scenario=%s simulation timeout (%.1f s), last_state=%s\n",
           scenario.name, time, mission_state_name(mission.state));
    return 3;
}
