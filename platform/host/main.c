/* Deterministic host simulation around the shared NavRuntime call chain. */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "scenario.h"
#include "sim_sensors.h"
#include "sim_vision.h"
#include "nav_telemetry.h"

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

static uint8_t write_telemetry_frame(FILE *file,
                                     const NavRuntimeOutput *output,
                                     uint32_t timestamp_ms,
                                     uint32_t *sequence)
{
    NavTelemetrySnapshot snapshot;
    uint8_t frame[NAV_TELEMETRY_FRAME_SIZE];
    if (file == 0) return 1u;
    if (!nav_telemetry_capture(&snapshot, output, *sequence, timestamp_ms) ||
        nav_telemetry_encode(&snapshot, frame) != NAV_TELEMETRY_OK ||
        fwrite(frame, 1u, NAV_TELEMETRY_FRAME_SIZE, file) !=
            NAV_TELEMETRY_FRAME_SIZE) {
        return 0u;
    }
    (*sequence)++;
    return 1u;
}

static int finish_with_telemetry(FILE *file, int result)
{
    if (file != 0 && fclose(file) != 0) {
        fprintf(stderr, "failed to close telemetry output\n");
        return result == 0 ? 7 : result;
    }
    return result;
}

int main(int argc, char **argv)
{
    Scenario scenario;
    SimState simulation;
    SimSensors sensors;
    SimVisionWorld vision_world;
    NavRuntimeConfig runtime_config;
    NavRuntime runtime;
    FILE *telemetry_file = 0;
    const char *telemetry_path = 0;
    uint32_t telemetry_sequence = 0u;
    float last_impact_time = -100.0f;
    float time = 0.0f;
    float max_position_error = 0.0f;
    int next_log_centi = 0;
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
        if (strcmp(argv[argument_index], "--scenario") == 0 &&
            argument_index + 1 < argc) {
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
        } else if (strcmp(argv[argument_index], "--telemetry") == 0 &&
                   argument_index + 1 < argc) {
            telemetry_path = argv[++argument_index];
        } else if (strcmp(argv[argument_index], "--seed") == 0 &&
                   argument_index + 1 < argc) {
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
    scenario_runtime_config(&scenario, &runtime_config);
    if (nav_runtime_init(&runtime, &runtime_config) != NAV_CONFIG_ERROR_NONE) {
        printf("configuration invalid: mask=0x%08x\n",
               (unsigned int)runtime.config_errors);
        return 1;
    }
    sim_vision_world_init(&vision_world, scenario.seed,
                          scenario.feature_area_m, scenario.feature_count);
    if (telemetry_path != 0) {
        telemetry_file = fopen(telemetry_path, "wb");
        if (telemetry_file == 0) {
            fprintf(stderr, "cannot open telemetry output: %s\n",
                    telemetry_path);
            return 1;
        }
    }

    printf("# RM Drone Nav Plan-B algorithm prototype\n");
    printf("# scenario=%s dt=%.3f budget=%.1fs seed=%u estimator=%s\n",
           scenario.name, scenario.dt, scenario.safety.max_mission_time_s,
           scenario.seed, scenario.estimator.mode == EST_MODE_INS ? "INS+VO" : "TRUTH");
    if (telemetry_file != 0) {
        printf("# telemetry=%s schema=%u frame_size=%u\n", telemetry_path,
               (unsigned int)NAV_TELEMETRY_SCHEMA_VERSION,
               (unsigned int)NAV_TELEMETRY_FRAME_SIZE);
    }

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
        FlowFrame flow_frame;
        PixelObs target_pixel;
        PixelObs home_pixel;
        SwarmView received_swarm;
        NavRuntimeInput input;
        SimImpact simulated_impact;
        float tof_height;
        uint8_t dock_contact;

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
            sim_vision_frame(&sensors, &vision_world, &simulation,
                             &scenario.cam_down, impact_visual_freeze,
                             time_ms, &flow_frame);
        } else {
            sim_sensors_vo(&sensors, &simulation, impact_visual_freeze,
                           scenario.dt, time_ms, &odometry);
        }

        swarm_view_init(&received_swarm, runtime_config.self_agent_id);
        if (scenario.other_agent_enabled) {
            scenario.other_agent.age_s = 0.0f;
            scenario.other_agent.timestamp_ms = time_ms;
            received_swarm.others[0] = scenario.other_agent;
            received_swarm.other_count = 1u;
        }

        dock_contact =
            (vec3_dist_xy(simulation.pos, scenario.home_pos) <= 0.12f &&
             simulation.pos.z <= 0.06f && vec3_norm(simulation.vel) <= 0.15f) ? 1u : 0u;

        input.timestamp_ms = time_ms;
        input.imu = &imu;
        input.flow = scenario.use_flow_vo ? &flow_frame : 0;
        input.odometry = scenario.use_flow_vo ? 0 : &odometry;
        input.tof_height = tof_height;
        input.target_pixel = &target_pixel;
        input.home_pixel = &home_pixel;
        input.obstacles = &scenario.obstacles;
        input.swarm = scenario.other_agent_enabled ? &received_swarm : 0;
        input.start_command = 1u;
        input.request_return = 0u;
        input.request_emergency = 0u;
        input.dock_contact = dock_contact;
        input.charging_detected =
            (dock_contact && runtime.mission.docking_stage == DOCK_CONTACT) ? 1u : 0u;
        input.wireless_charge_ready = dock_contact;
        input.dt = scenario.dt;

        if (!nav_runtime_step(&runtime, &input)) {
            if (!write_telemetry_frame(telemetry_file, &runtime.output,
                                       time_ms, &telemetry_sequence)) {
                fprintf(stderr, "failed to write telemetry output\n");
                return finish_with_telemetry(telemetry_file, 7);
            }
            printf("MISSION_FAILED scenario=%s runtime rejected input\n", scenario.name);
            return finish_with_telemetry(telemetry_file, 5);
        }
        if (!write_telemetry_frame(telemetry_file, &runtime.output,
                                   time_ms, &telemetry_sequence)) {
            fprintf(stderr, "failed to write telemetry output\n");
            return finish_with_telemetry(telemetry_file, 7);
        }

        if ((runtime.output.event_flags & NAV_EVENT_IMPACT_CONFIRMED) != 0u) {
            printf("[t=%6.2f] EVENT impact confirmed confidence=%.2f\n",
                   time, runtime.output.impact.confidence);
        }
        if (target_hidden &&
            (runtime.output.mission.state == MS_TARGET_TRACK ||
             runtime.output.mission.state == MS_TERMINAL)) {
            if (!target_loss_exercised) {
                printf("[t=%6.2f] EVENT target observation interrupted\n", time);
            }
            target_loss_exercised = 1u;
        }
        if (home_hidden &&
            (runtime.output.mission.state == MS_HOME_SEARCH ||
             runtime.output.mission.state == MS_HOMING ||
             runtime.output.mission.state == MS_DOCKING)) {
            if (!home_loss_exercised) {
                printf("[t=%6.2f] EVENT home marker unavailable\n", time);
            }
            home_loss_exercised = 1u;
        }
        if (runtime.output.recovery_active &&
            (runtime.output.nav.status == EST_DEGRADED ||
             runtime.output.nav.status == EST_LOST ||
             runtime.output.nav.status == EST_RECOVERING)) {
            recovery_degradation_exercised = 1u;
        }
        if ((runtime.output.event_flags & NAV_EVENT_OBSTACLE_RISK) != 0u) {
            if (!obstacle_avoidance_exercised) {
                printf("[t=%6.2f] EVENT local obstacle risk id=%u clearance=%.2f m\n",
                       time, runtime.output.obstacle.obstacle_id,
                       runtime.output.obstacle.minimum_separation_m);
            }
            obstacle_avoidance_exercised = 1u;
        }
        if ((runtime.output.event_flags & NAV_EVENT_SWARM_CONFLICT) != 0u) {
            if (!swarm_conflict_exercised) {
                printf("[t=%6.2f] EVENT swarm conflict agent=%u separation=%.2f m\n",
                       time, runtime.output.swarm_collision.other_agent_id,
                       runtime.output.swarm_collision.min_separation);
            }
            swarm_conflict_exercised = 1u;
        }
        if ((runtime.output.event_flags & NAV_EVENT_TRAJECTORY_DETOUR) != 0u) {
            if (!trajectory_detour_exercised) {
                printf("[t=%6.2f] EVENT fixed-memory trajectory detour selected\n", time);
            }
            trajectory_detour_exercised = 1u;
        }
        if ((runtime.output.event_flags & NAV_EVENT_TRAJECTORY_INVALID) != 0u) {
            printf("[t=%6.2f] EVENT trajectory rejected flags=0x%02x; using bounded fallback\n",
                   time, runtime.output.trajectory.check.flags);
        }

        if (runtime.output.mission.state_changed) {
            printf("[t=%6.2f] STATE -> %s est=%s safety=%s\n",
                   time, mission_state_name(runtime.output.mission.state),
                   est_status_name(runtime.output.nav.status),
                   safety_level_name(runtime.output.safety.level));
            if (runtime.output.mission.state == MS_RETURN_HOME) {
                forced_return_exercised = 1u;
            }
        }

        simulated_impact.active = 0u;
        simulated_impact.delta_v = vec3_zero();
        simulated_impact.delta_yaw = 0.0f;
        simulated_impact.delta_pitch = 0.0f;
        simulated_impact.delta_roll = 0.0f;
        if (runtime.output.mission.state == MS_TERMINAL &&
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
        sim_dynamics_step(&simulation, &runtime.output.control, &scenario.dynamics,
                          &simulated_impact, scenario.dt);
        scenario.target_pos = vec3_add(scenario.target_pos,
                                       vec3_scale(scenario.target_velocity, scenario.dt));
        if (scenario.other_agent_enabled) {
            scenario.other_agent.pos = vec3_add(scenario.other_agent.pos,
                vec3_scale(scenario.other_agent.vel, scenario.dt));
        }

        if (runtime.output.nav.status != EST_LOST) {
            float error = vec3_dist(runtime.output.nav.pos, simulation.pos);
            if (error > max_position_error) max_position_error = error;
        }

        if ((int)(time * 100.0f) >= next_log_centi) {
            next_log_centi += 100;
            printf("[t=%6.2f] %-16s est=%-12s safety=%-17s pos=(%5.2f,%5.2f,%4.2f) "
                   "target=%-8s term=%-16s recovery=%-18s home=%c dock=%s\n",
                   time, mission_state_name(runtime.output.mission.state),
                   est_status_name(runtime.output.nav.status),
                   safety_level_name(runtime.output.safety.level),
                   runtime.output.nav.pos.x, runtime.output.nav.pos.y,
                   runtime.output.nav.pos.z,
                   target_track_status_name(runtime.output.target.status),
                   terminal_stage_name(runtime.output.terminal_stage),
                   recovery_stage_name(runtime.output.recovery.stage),
                   runtime.output.home.visible ? 'Y' : 'N',
                   docking_stage_name(runtime.output.mission.docking_stage));
        }

        if (runtime.output.mission.mission_complete) {
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
                return finish_with_telemetry(telemetry_file, 4);
            }
            printf("[t=%6.2f] MISSION_SUCCESS scenario=%s dist_to_home=%.3f m "
                   "elapsed=%.2f s remaining=%.2f s max_est_err=%.2f m\n",
                   time, scenario.name, vec3_dist(simulation.pos, scenario.home_pos),
                   time, runtime.output.safety.remaining_s, max_position_error);
            return finish_with_telemetry(telemetry_file, 0);
        }
        if (runtime.output.mission.mission_failed) {
            printf("[t=%6.2f] MISSION_FAILED scenario=%s emergency landed "
                   "max_est_err=%.2f m\n",
                   time, scenario.name, max_position_error);
            return finish_with_telemetry(telemetry_file, 2);
        }
        time += scenario.dt;
    }

    printf("MISSION_FAILED scenario=%s simulation timeout (%.1f s), last_state=%s\n",
           scenario.name, time, mission_state_name(runtime.output.mission.state));
    return finish_with_telemetry(telemetry_file, 3);
}
