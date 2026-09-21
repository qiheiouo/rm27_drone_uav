/* Deterministic multi-NavRuntime simulation over the real swarm wire format. */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "nav_runtime.h"
#include "sim_dynamics.h"
#include "swarm_link.h"
#include "swarm_network.h"

#define SWARM_SIM_MAX_AGENTS SIM_SWARM_NETWORK_MAX_NODES
#define SWARM_SIM_DT_S 0.02f
#define SWARM_SIM_STEP_MS 20u
#define SWARM_SIM_TX_PERIOD_STEPS 5u
#define SWARM_SIM_PEER_MAX_AGE_MS 350u

typedef enum {
    SWARM_SIM_NOMINAL = 0,
    SWARM_SIM_LOSSY,
    SWARM_SIM_OUTAGE,
    SWARM_SIM_GUARDED_OUTAGE,
    SWARM_SIM_FOUR_AGENT
} SwarmSimScenario;

typedef struct {
    uint8_t agent_id;
    NavRuntime runtime;
    SimState truth;
    SwarmPeerRegistry peers;
    SwarmView view;
    uint32_t conflicts;
    uint32_t yielding_steps;
    uint32_t guard_holding_steps;
    uint32_t guard_recovering_steps;
    uint32_t guard_reason_steps;
    uint32_t first_conflict_ms;
    uint32_t decode_errors;
    uint8_t max_peer_count;
    uint8_t peer_seen_before_outage;
    uint8_t peer_expired_during_outage;
    uint8_t peer_reacquired;
    uint8_t guard_recovery_seen;
    uint8_t guard_recovered;
    uint8_t guard_guidance_invalid;
    uint8_t emergency_seen;
} SwarmSimAgent;

static const char *scenario_name(SwarmSimScenario scenario)
{
    switch (scenario) {
    case SWARM_SIM_NOMINAL: return "nominal";
    case SWARM_SIM_LOSSY: return "lossy";
    case SWARM_SIM_OUTAGE: return "outage";
    case SWARM_SIM_GUARDED_OUTAGE: return "guarded-outage";
    case SWARM_SIM_FOUR_AGENT: return "four-agent";
    default: return "unknown";
    }
}

static int parse_scenario(const char *name, SwarmSimScenario *scenario)
{
    if (strcmp(name, "nominal") == 0) {
        *scenario = SWARM_SIM_NOMINAL;
    } else if (strcmp(name, "lossy") == 0) {
        *scenario = SWARM_SIM_LOSSY;
    } else if (strcmp(name, "outage") == 0) {
        *scenario = SWARM_SIM_OUTAGE;
    } else if (strcmp(name, "guarded-outage") == 0) {
        *scenario = SWARM_SIM_GUARDED_OUTAGE;
    } else if (strcmp(name, "four-agent") == 0) {
        *scenario = SWARM_SIM_FOUR_AGENT;
    } else {
        return -1;
    }
    return 0;
}

static void print_scenarios(void)
{
    printf("nominal\nlossy\noutage\nguarded-outage\nfour-agent\n");
}

static uint8_t output_finite(const NavRuntimeOutput *output)
{
    return (vec3_is_finite(output->nav.pos) &&
            vec3_is_finite(output->nav.vel) &&
            vec3_is_finite(output->guidance.vel_sp) &&
            vec3_is_finite(output->control.accel_cmd) &&
            nav_isfinite(output->control.yaw_rate_cmd)) ? 1u : 0u;
}

static void crossing_geometry(uint8_t index, Vec3f *home, Vec3f *goal)
{
    if (index == 0u) {
        *home = vec3(-1.6f, 0.0f, 0.0f);
        *goal = vec3(1.6f, 0.0f, 1.2f);
    } else {
        *home = vec3(0.0f, -1.6f, 0.0f);
        *goal = vec3(0.0f, 1.6f, 1.2f);
    }
}

static void parallel_geometry(uint8_t index, Vec3f *home, Vec3f *goal)
{
    float y = -1.5f + (float)index;
    *home = vec3(-1.6f, y, 0.0f);
    *goal = vec3(1.6f, y, 1.2f);
}

static uint8_t agent_init(SwarmSimAgent *agent,
                          uint8_t index,
                          SwarmSimScenario scenario)
{
    NavRuntimeConfig cfg;
    Vec3f home;
    Vec3f goal;

    if (scenario == SWARM_SIM_FOUR_AGENT) {
        parallel_geometry(index, &home, &goal);
    } else {
        crossing_geometry(index, &home, &goal);
    }

    memset(agent, 0, sizeof(*agent));
    agent->agent_id = (uint8_t)(index + 1u);
    nav_runtime_config_default(&cfg);
    cfg.self_agent_id = agent->agent_id;
    cfg.nominal_dt_s = SWARM_SIM_DT_S;
    cfg.estimator.mode = EST_MODE_TRUTH;
    cfg.home_pos = home;
    cfg.mission.self_check_s = 0.08f;
    cfg.mission.docked_launch_delay_s = 0.08f;
    cfg.mission.takeoff_alt_m = 1.2f;
    cfg.safety.max_mission_time_s = 18.0f;
    cfg.safety.soft_return_deadline_s = 14.0f;
    cfg.safety.hard_return_deadline_s = 17.0f;
    cfg.swarm_collision.safe_separation_m = 0.80f;
    cfg.swarm_collision.critical_separation_m = 0.18f;
    cfg.swarm_collision.prediction_horizon_s = 2.0f;
    cfg.swarm_collision.max_message_age_s =
        (float)SWARM_SIM_PEER_MAX_AGE_MS * 0.001f;
    cfg.swarm_avoidance.mode = SWARM_ENABLED;
    cfg.swarm_avoidance.lateral_bias_mps = 0.85f;
    cfg.swarm_avoidance.critical_climb_mps = 0.40f;
    if (scenario == SWARM_SIM_GUARDED_OUTAGE) {
        cfg.swarm_link_guard.enabled = 1u;
    }
    wq_init(&cfg.outbound_route);
    (void)wq_push(&cfg.outbound_route, goal, 1.25f);
    wq_init(&cfg.search_route);
    (void)wq_push(&cfg.search_route, goal, 0.6f);

    if (nav_runtime_init(&agent->runtime, &cfg) != NAV_CONFIG_ERROR_NONE) {
        printf("agent %u configuration invalid: 0x%08x\n",
               agent->agent_id, (unsigned int)agent->runtime.config_errors);
        return 0u;
    }
    sim_dynamics_init(&agent->truth, home, 0.0f);
    swarm_peer_registry_init(&agent->peers, agent->agent_id,
                             SWARM_SIM_PEER_MAX_AGE_MS);
    swarm_view_init(&agent->view, agent->agent_id);
    return 1u;
}

static void network_config(SwarmSimScenario scenario,
                           SimSwarmNetworkConfig *cfg)
{
    sim_swarm_network_config_default(cfg);
    cfg->seed = 2027u;
    if (scenario == SWARM_SIM_LOSSY) {
        cfg->base_delay_ms = 40u;
        cfg->jitter_ms = 60u;
        cfg->drop_every_n = 4u;
        cfg->duplicate_every_n = 5u;
        cfg->reorder_every_n = 3u;
        cfg->reorder_extra_delay_ms = 160u;
    } else if (scenario == SWARM_SIM_OUTAGE) {
        cfg->base_delay_ms = 20u;
        cfg->outage_start_ms = 1600u;
        cfg->outage_end_ms = 2500u;
    }
}

static void receive_frames(SwarmSimAgent *agent,
                           SimSwarmNetwork *network,
                           uint32_t now_ms)
{
    uint8_t frame[SWARM_LINK_STATE_FRAME_SIZE];
    uint16_t size;
    uint8_t source_id;
    int result;

    do {
        AgentState state;
        result = sim_swarm_network_receive(network, agent->agent_id, now_ms,
            frame, sizeof(frame), &size, &source_id);
        if (result == 1) {
            if (swarm_link_decode_state(frame, size, &state) != SWARM_LINK_OK ||
                state.agent_id != source_id) {
                agent->decode_errors++;
            } else {
                (void)swarm_peer_registry_ingest(&agent->peers, &state, now_ms);
            }
        }
    } while (result == 1);
}

static uint8_t step_agent(SwarmSimAgent *agent, uint32_t now_ms)
{
    ImuSample imu;
    OdomSample odometry;
    NavRuntimeInput input;

    imu.accel = agent->truth.spec_force_body;
    imu.gyro = agent->truth.gyro_body;
    imu.timestamp_ms = now_ms;
    odometry.pos = agent->truth.pos;
    odometry.vel = agent->truth.vel;
    odometry.yaw = agent->truth.yaw;
    odometry.yaw_rate = agent->truth.yaw_rate;
    odometry.att = agent->truth.att;
    odometry.valid = 1u;
    odometry.timestamp_ms = now_ms;

    memset(&input, 0, sizeof(input));
    input.timestamp_ms = now_ms;
    input.imu = &imu;
    input.odometry = &odometry;
    input.tof_height = agent->truth.pos.z;
    input.swarm = &agent->view;
    input.start_command = 1u;
    input.dt = SWARM_SIM_DT_S;
    return nav_runtime_step(&agent->runtime, &input);
}

static void update_outage_observation(SwarmSimAgent *agent,
                                      const SimSwarmNetworkConfig *network_cfg,
                                      uint32_t now_ms)
{
    if (network_cfg->outage_start_ms >= network_cfg->outage_end_ms) return;
    if (now_ms < network_cfg->outage_start_ms && agent->view.other_count > 0u) {
        agent->peer_seen_before_outage = 1u;
    }
    if (now_ms >= network_cfg->outage_start_ms + SWARM_SIM_PEER_MAX_AGE_MS &&
        now_ms < network_cfg->outage_end_ms &&
        agent->view.other_count == 0u) {
        agent->peer_expired_during_outage = 1u;
    }
    if (now_ms >= network_cfg->outage_end_ms + network_cfg->base_delay_ms +
                  SWARM_SIM_STEP_MS &&
        agent->peer_expired_during_outage && agent->view.other_count > 0u) {
        agent->peer_reacquired = 1u;
    }
}

static uint8_t runtime_log_contains(const NavRuntime *runtime,
                                    NavLogEventCode code)
{
    NavEventRecord record;
    uint16_t index;
    const NavEventLog *log = nav_runtime_event_log(runtime);
    for (index = 0u; index < nav_event_log_count(log); index++) {
        if (nav_event_log_get(log, index, &record) && record.code == code) {
            return 1u;
        }
    }
    return 0u;
}

static uint8_t verify_results(SwarmSimScenario scenario,
                              const SwarmSimAgent *agents,
                              uint8_t agent_count,
                              const SimSwarmNetwork *network,
                              float minimum_separation)
{
    uint32_t duplicate_frames = 0u;
    uint32_t out_of_order_frames = 0u;
    uint8_t expected_peers = (uint8_t)(agent_count - 1u);
    uint8_t index;

    if (network->stats.delivered_frames == 0u ||
        network->stats.queue_overflows != 0u ||
        sim_swarm_network_pending(network) >= SIM_SWARM_NETWORK_QUEUE_CAPACITY) {
        return 0u;
    }
    for (index = 0u; index < agent_count; index++) {
        const SwarmSimAgent *agent = &agents[index];
        if (agent->decode_errors != 0u || agent->emergency_seen ||
            agent->max_peer_count != expected_peers ||
            agent->peers.stats.capacity_drops != 0u) {
            return 0u;
        }
        duplicate_frames += agent->peers.stats.duplicate_frames;
        out_of_order_frames += agent->peers.stats.out_of_order_frames;
    }

    if (scenario == SWARM_SIM_FOUR_AGENT) {
        return minimum_separation > 0.80f ? 1u : 0u;
    }
    if (agents[0].conflicts == 0u || agents[1].conflicts == 0u ||
        agents[0].yielding_steps != 0u || agents[1].yielding_steps == 0u ||
        minimum_separation <=
            (scenario == SWARM_SIM_OUTAGE ? 0.10f : 0.18f)) {
        return 0u;
    }
    if (scenario == SWARM_SIM_LOSSY &&
        (network->stats.dropped_frames == 0u ||
         network->stats.duplicated_frames == 0u ||
         network->stats.reordered_frames == 0u ||
         duplicate_frames == 0u || out_of_order_frames == 0u)) {
        return 0u;
    }
    if (scenario == SWARM_SIM_OUTAGE ||
        scenario == SWARM_SIM_GUARDED_OUTAGE) {
        if (network->stats.outage_drops == 0u) return 0u;
        for (index = 0u; index < agent_count; index++) {
            if (!agents[index].peer_seen_before_outage ||
                !agents[index].peer_expired_during_outage ||
                !agents[index].peer_reacquired ||
                agents[index].peers.stats.expired_peers == 0u) {
                return 0u;
            }
        }
    }
    if (scenario == SWARM_SIM_GUARDED_OUTAGE) {
        for (index = 0u; index < agent_count; index++) {
            if (agents[index].guard_holding_steps == 0u ||
                agents[index].guard_recovering_steps == 0u ||
                agents[index].guard_reason_steps == 0u ||
                !agents[index].guard_recovered ||
                agents[index].guard_guidance_invalid ||
                !runtime_log_contains(&agents[index].runtime,
                                      NAV_LOG_SWARM_LINK_HOLD) ||
                !runtime_log_contains(&agents[index].runtime,
                                      NAV_LOG_SWARM_LINK_RECOVERED)) {
                return 0u;
            }
        }
    }
    return 1u;
}

int main(int argc, char **argv)
{
    SwarmSimScenario scenario = SWARM_SIM_NOMINAL;
    SimSwarmNetworkConfig network_cfg;
    SimSwarmNetwork network;
    SwarmSimAgent agents[SWARM_SIM_MAX_AGENTS];
    SimParams dynamics;
    SimImpact no_impact;
    uint8_t agent_count;
    uint32_t duration_ms;
    uint32_t step;
    float minimum_separation = 1e9f;
    uint8_t index;
    uint8_t guarded_outage_scheduled = 0u;
    int argument_index;

    for (argument_index = 1; argument_index < argc; argument_index++) {
        if (strcmp(argv[argument_index], "--scenario") == 0 &&
            argument_index + 1 < argc) {
            if (parse_scenario(argv[++argument_index], &scenario) != 0) {
                printf("unknown scenario: %s\n", argv[argument_index]);
                print_scenarios();
                return 1;
            }
        } else if (strcmp(argv[argument_index], "--list-scenarios") == 0) {
            print_scenarios();
            return 0;
        } else {
            printf("unknown argument: %s\n", argv[argument_index]);
            return 1;
        }
    }

    agent_count = scenario == SWARM_SIM_FOUR_AGENT ? 4u : 2u;
    duration_ms = scenario == SWARM_SIM_FOUR_AGENT ? 5000u : 7000u;
    network_config(scenario, &network_cfg);
    sim_swarm_network_init(&network, &network_cfg);
    for (index = 0u; index < agent_count; index++) {
        if (!agent_init(&agents[index], index, scenario) ||
            sim_swarm_network_add_node(&network, agents[index].agent_id) !=
                SIM_SWARM_NETWORK_OK) {
            return 1;
        }
    }

    dynamics.drag = 0.5f;
    dynamics.max_speed = 2.5f;
    dynamics.max_yaw_rate = 3.0f;
    dynamics.max_att_rate = 10.0f;
    memset(&no_impact, 0, sizeof(no_impact));
    printf("# deterministic swarm simulation scenario=%s agents=%u\n",
           scenario_name(scenario), agent_count);

    for (step = 1u; step * SWARM_SIM_STEP_MS <= duration_ms; step++) {
        uint32_t now_ms = step * SWARM_SIM_STEP_MS;

        for (index = 0u; index < agent_count; index++) {
            receive_frames(&agents[index], &network, now_ms);
            swarm_peer_registry_build_view(&agents[index].peers, 0, now_ms,
                                           &agents[index].view);
            if (agents[index].view.other_count > agents[index].max_peer_count) {
                agents[index].max_peer_count = agents[index].view.other_count;
            }
            update_outage_observation(&agents[index], &network_cfg, now_ms);
        }

        for (index = 0u; index < agent_count; index++) {
            SwarmSimAgent *agent = &agents[index];
            if (!step_agent(agent, now_ms) ||
                !output_finite(&agent->runtime.output)) {
                printf("SWARM_SIM_FAILED scenario=%s agent=%u invalid runtime output\n",
                       scenario_name(scenario), agent->agent_id);
                return 2;
            }
            if (agent->runtime.output.swarm_collision.conflict) {
                agent->conflicts++;
                if (agent->first_conflict_ms == 0u) {
                    agent->first_conflict_ms = now_ms;
                }
            }
            if (agent->runtime.output.swarm_avoidance.yielding) {
                agent->yielding_steps++;
            }
            if (agent->runtime.output.swarm_link_guard.state ==
                SWARM_LINK_GUARD_HOLDING) {
                agent->guard_holding_steps++;
            }
            if (agent->runtime.output.swarm_link_guard.state ==
                SWARM_LINK_GUARD_RECOVERING) {
                agent->guard_recovering_steps++;
                agent->guard_recovery_seen = 1u;
            }
            if (agent->guard_recovery_seen &&
                agent->runtime.output.swarm_link_guard.state ==
                    SWARM_LINK_GUARD_CLEAR) {
                agent->guard_recovered = 1u;
            }
            if ((agent->runtime.output.safety.reason_mask &
                 SAFETY_REASON_SWARM_LINK) != 0u) {
                agent->guard_reason_steps++;
            }
            if (agent->runtime.output.swarm_link_guard.active &&
                (!agent->runtime.output.guidance.use_pos_sp ||
                 agent->runtime.output.trajectory_active ||
                 vec3_dist(agent->runtime.output.guidance.pos_sp,
                    agent->runtime.output.swarm_link_guard.hold_position) >
                        1e-4f)) {
                agent->guard_guidance_invalid = 1u;
            }
            if (agent->runtime.output.mission.state == MS_EMERGENCY_STABILIZE ||
                agent->runtime.output.mission.state == MS_EMERGENCY_LAND ||
                agent->runtime.output.mission.state == MS_ESTIMATOR_LOST) {
                agent->emergency_seen = 1u;
            }
        }

        if (scenario == SWARM_SIM_GUARDED_OUTAGE &&
            !guarded_outage_scheduled &&
            agents[0].runtime.output.swarm_collision.conflict &&
            agents[1].runtime.output.swarm_collision.conflict) {
            network_cfg.outage_start_ms = now_ms;
            network_cfg.outage_end_ms = now_ms + 1000u;
            network.cfg.outage_start_ms = network_cfg.outage_start_ms;
            network.cfg.outage_end_ms = network_cfg.outage_end_ms;
            agents[0].peer_seen_before_outage =
                agents[0].view.other_count > 0u ? 1u : 0u;
            agents[1].peer_seen_before_outage =
                agents[1].view.other_count > 0u ? 1u : 0u;
            guarded_outage_scheduled = 1u;
            printf("[t=%.2f] EVENT guarded conflict-link outage scheduled\n",
                   (float)now_ms * 0.001f);
        }

        for (index = 0u; index < agent_count; index++) {
            sim_dynamics_step(&agents[index].truth,
                &agents[index].runtime.output.control, &dynamics,
                &no_impact, SWARM_SIM_DT_S);
        }

        if (step % SWARM_SIM_TX_PERIOD_STEPS == 0u) {
            for (index = 0u; index < agent_count; index++) {
                uint8_t frame[SWARM_LINK_STATE_FRAME_SIZE];
                if (swarm_link_encode_state(
                        &agents[index].runtime.output.swarm_self, frame) !=
                        SWARM_LINK_OK ||
                    sim_swarm_network_broadcast(&network,
                        agents[index].agent_id, frame, sizeof(frame), now_ms) ==
                        SIM_SWARM_NETWORK_ERROR_CAPACITY) {
                    printf("SWARM_SIM_FAILED scenario=%s network queue overflow\n",
                           scenario_name(scenario));
                    return 3;
                }
            }
        }

        for (index = 0u; index < agent_count; index++) {
            uint8_t other_index;
            if (agents[index].truth.pos.z <= 0.8f) continue;
            for (other_index = (uint8_t)(index + 1u);
                 other_index < agent_count; other_index++) {
                float separation;
                if (agents[other_index].truth.pos.z <= 0.8f) continue;
                separation = vec3_dist(agents[index].truth.pos,
                                       agents[other_index].truth.pos);
                if (separation < minimum_separation) {
                    minimum_separation = separation;
                }
            }
        }
    }

    for (index = 0u; index < agent_count; index++) {
        printf("agent=%u peers=%u conflicts=%u yielding=%u guard_hold=%u "
               "guard_recover=%u accepted=%u duplicate=%u "
               "out_of_order=%u expired=%u first_conflict_ms=%u\n",
               agents[index].agent_id, agents[index].max_peer_count,
               (unsigned int)agents[index].conflicts,
               (unsigned int)agents[index].yielding_steps,
               (unsigned int)agents[index].guard_holding_steps,
               (unsigned int)agents[index].guard_recovering_steps,
               (unsigned int)agents[index].peers.stats.accepted_frames,
               (unsigned int)agents[index].peers.stats.duplicate_frames,
               (unsigned int)agents[index].peers.stats.out_of_order_frames,
               (unsigned int)agents[index].peers.stats.expired_peers,
               (unsigned int)agents[index].first_conflict_ms);
    }
    printf("network delivered=%u dropped=%u duplicated=%u reordered=%u "
           "outage_drops=%u pending=%u min_separation=%.3f\n",
           (unsigned int)network.stats.delivered_frames,
           (unsigned int)network.stats.dropped_frames,
           (unsigned int)network.stats.duplicated_frames,
           (unsigned int)network.stats.reordered_frames,
           (unsigned int)network.stats.outage_drops,
           sim_swarm_network_pending(&network), minimum_separation);

    if (!verify_results(scenario, agents, agent_count, &network,
                        minimum_separation)) {
        printf("SWARM_SIM_FAILED scenario=%s expectations not met\n",
               scenario_name(scenario));
        return 4;
    }
    printf("SWARM_SIM_SUCCESS scenario=%s agents=%u\n",
           scenario_name(scenario), agent_count);
    return 0;
}
