#include <string.h>
#include "scenario.h"

void scenario_default(Scenario *scenario)
{
    NavRuntimeConfig runtime_cfg;

    nav_runtime_config_default(&runtime_cfg);
    scenario->kind = SCENARIO_NOMINAL;
    scenario->name = "nominal";
    scenario->home_pos = runtime_cfg.home_pos;
    scenario->target_pos = vec3(4.0f, 2.0f, 1.0f);
    scenario->target_velocity = vec3_zero();
    scenario->outbound_route = runtime_cfg.outbound_route;
    scenario->search_route = runtime_cfg.search_route;
    scenario->mission = runtime_cfg.mission;
    scenario->estimator = runtime_cfg.estimator;
    scenario->impact = runtime_cfg.impact;
    scenario->impact_fusion = runtime_cfg.impact_fusion;
    scenario->recovery = runtime_cfg.recovery;
    scenario->safety = runtime_cfg.safety;
    scenario->health = runtime_cfg.health;
    scenario->ctrl = runtime_cfg.ctrl;
    scenario->terminal = runtime_cfg.terminal;
    scenario->home_guidance = runtime_cfg.home_guidance;
    scenario->planner = runtime_cfg.planner;
    scenario->obstacle_avoidance = runtime_cfg.obstacle_avoidance;
    scenario->swarm_collision = runtime_cfg.swarm_collision;
    scenario->swarm_avoidance = runtime_cfg.swarm_avoidance;
    scenario->cam_forward = runtime_cfg.cam_forward;
    scenario->cam_down = runtime_cfg.cam_down;
    scenario->target_size_m = runtime_cfg.target_size_m;
    scenario->marker_size_m = runtime_cfg.marker_size_m;
    scenario->flow = runtime_cfg.flow;

    scenario->dynamics.drag = 0.5f;
    scenario->dynamics.max_speed = 2.5f;
    scenario->dynamics.max_yaw_rate = 3.0f;
    scenario->dynamics.max_att_rate = 10.0f;
    obstacle_set_init(&scenario->obstacles);
    scenario->use_flow_vo = 1u;
    scenario->feature_area_m = 14.0f;
    scenario->feature_count = 4000u;

    scenario->dt = 0.01f;
    scenario->sim_max_time_s = 60.0f;
    scenario->impact_range_m = 0.25f;
    scenario->vision_freeze_s = 0.4f;
    scenario->seed = 12345u;
    scenario->impact_delta_v = vec3(-1.8f, 1.2f, 0.9f);
    scenario->impact_delta_yaw = 0.9f;
    scenario->impact_delta_pitch = 0.5f;
    scenario->impact_delta_roll = 0.4f;

    scenario->target_hidden_start_s = -1.0f;
    scenario->target_hidden_end_s = -1.0f;
    scenario->home_hidden_until_s = -1.0f;
    scenario->home_loss_start_s = -1.0f;
    scenario->home_loss_end_s = -1.0f;
    scenario->other_agent_enabled = 0u;
    agent_state_clear(&scenario->other_agent);
}

void scenario_runtime_config(const Scenario *scenario, NavRuntimeConfig *cfg)
{
    nav_runtime_config_default(cfg);
    cfg->mission = scenario->mission;
    cfg->estimator = scenario->estimator;
    cfg->impact = scenario->impact;
    cfg->impact_fusion = scenario->impact_fusion;
    cfg->recovery = scenario->recovery;
    cfg->safety = scenario->safety;
    cfg->health = scenario->health;
    cfg->obstacle_avoidance = scenario->obstacle_avoidance;
    cfg->planner = scenario->planner;
    cfg->swarm_collision = scenario->swarm_collision;
    cfg->swarm_avoidance = scenario->swarm_avoidance;
    cfg->ctrl = scenario->ctrl;
    cfg->terminal = scenario->terminal;
    cfg->home_guidance = scenario->home_guidance;
    cfg->flow = scenario->flow;
    cfg->cam_forward = scenario->cam_forward;
    cfg->cam_down = scenario->cam_down;
    cfg->target_size_m = scenario->target_size_m;
    cfg->marker_size_m = scenario->marker_size_m;
    cfg->nominal_dt_s = scenario->dt;
    cfg->home_pos = scenario->home_pos;
    cfg->outbound_route = scenario->outbound_route;
    cfg->search_route = scenario->search_route;
}

int scenario_apply_kind(Scenario *scenario, const char *name)
{
    DynamicObstacle obstacle;
    if (strcmp(name, "nominal") == 0) {
        scenario->kind = SCENARIO_NOMINAL;
    } else if (strcmp(name, "moving-target") == 0) {
        scenario->kind = SCENARIO_MOVING_TARGET;
        scenario->target_velocity = vec3(0.0f, 0.08f, 0.0f);
    } else if (strcmp(name, "target-loss") == 0) {
        scenario->kind = SCENARIO_TARGET_LOSS;
        scenario->target_hidden_start_s = 7.2f;
        scenario->target_hidden_end_s = 7.8f;
    } else if (strcmp(name, "impact-degraded") == 0) {
        scenario->kind = SCENARIO_IMPACT_DEGRADED;
        scenario->vision_freeze_s = 1.0f;
        scenario->estimator.impact_blind_s = 1.0f;
    } else if (strcmp(name, "impact-lost") == 0) {
        scenario->kind = SCENARIO_IMPACT_LOST;
        scenario->estimator.lost_on_impact = 1u;
        scenario->vision_freeze_s = 0.9f;
        scenario->estimator.impact_blind_s = 0.9f;
    } else if (strcmp(name, "home-initial-hidden") == 0) {
        scenario->kind = SCENARIO_HOME_INITIAL_HIDDEN;
        scenario->home_hidden_until_s = 19.0f;
    } else if (strcmp(name, "home-loss") == 0) {
        scenario->kind = SCENARIO_HOME_LOSS;
        scenario->home_loss_start_s = 19.0f;
        scenario->home_loss_end_s = 20.0f;
    } else if (strcmp(name, "local-obstacle") == 0) {
        scenario->kind = SCENARIO_LOCAL_OBSTACLE;
        /* Exercise the planner detour on the active edge immediately. */
        wq_init(&scenario->outbound_route);
        (void)wq_push(&scenario->outbound_route,
                      vec3(3.5f, 0.5f, 1.2f), 1.8f);
        obstacle.valid = 1u;
        obstacle.obstacle_id = 1u;
        obstacle.position = vec3(2.7f, 0.2f, 1.2f);
        obstacle.velocity = vec3_zero();
        obstacle.radius_m = 0.25f;
        obstacle.confidence = 1.0f;
        obstacle.age_s = 0.0f;
        obstacle_set_push(&scenario->obstacles, &obstacle);
    } else if (strcmp(name, "forced-return") == 0) {
        scenario->kind = SCENARIO_FORCED_RETURN;
        scenario->safety.soft_return_deadline_s = 5.0f;
        scenario->safety.hard_return_deadline_s = 30.0f;
    } else if (strcmp(name, "two-agent-conflict") == 0) {
        scenario->kind = SCENARIO_TWO_AGENT_CONFLICT;
        scenario->other_agent_enabled = 1u;
        scenario->other_agent.agent_id = 1u;
        scenario->other_agent.valid = 1u;
        scenario->other_agent.quality = 1.0f;
        scenario->other_agent.age_s = 0.0f;
        /* The higher-ID vehicle yields to +y while the peer crosses from -y. */
        scenario->other_agent.pos = vec3(1.8f, -1.1f, 1.2f);
        scenario->other_agent.vel = vec3(0.0f, 0.20f, 0.0f);
        scenario->swarm_avoidance.mode = SWARM_ENABLED;
    } else {
        return -1;
    }
    scenario->name = scenario_kind_name(scenario->kind);
    return 0;
}

const char *scenario_kind_name(ScenarioKind kind)
{
    switch (kind) {
    case SCENARIO_NOMINAL: return "nominal";
    case SCENARIO_MOVING_TARGET: return "moving-target";
    case SCENARIO_TARGET_LOSS: return "target-loss";
    case SCENARIO_IMPACT_DEGRADED: return "impact-degraded";
    case SCENARIO_IMPACT_LOST: return "impact-lost";
    case SCENARIO_HOME_INITIAL_HIDDEN: return "home-initial-hidden";
    case SCENARIO_HOME_LOSS: return "home-loss";
    case SCENARIO_LOCAL_OBSTACLE: return "local-obstacle";
    case SCENARIO_FORCED_RETURN: return "forced-return";
    case SCENARIO_TWO_AGENT_CONFLICT: return "two-agent-conflict";
    default: return "unknown";
    }
}
