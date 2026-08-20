#include <string.h>
#include "scenario.h"

void scenario_default(Scenario *scenario)
{
    scenario->kind = SCENARIO_NOMINAL;
    scenario->name = "nominal";
    scenario->home_pos = vec3(0.0f, 0.0f, 0.0f);
    scenario->target_pos = vec3(4.0f, 2.0f, 1.0f);
    scenario->target_velocity = vec3_zero();

    wq_init(&scenario->outbound_route);
    wq_push(&scenario->outbound_route, vec3(2.0f, 0.0f, 1.2f), 1.8f);
    wq_push(&scenario->outbound_route, vec3(3.5f, 0.5f, 1.2f), 1.8f);
    wq_init(&scenario->search_route);
    wq_push(&scenario->search_route, vec3(3.5f, 1.5f, 1.2f), 1.2f);
    wq_push(&scenario->search_route, vec3(4.5f, 1.5f, 1.2f), 1.2f);
    wq_push(&scenario->search_route, vec3(4.5f, 2.5f, 1.2f), 1.2f);
    wq_push(&scenario->search_route, vec3(3.5f, 2.5f, 1.2f), 1.2f);

    scenario->mission.takeoff_alt_m = 1.2f;
    scenario->mission.waypoint_tol_m = 0.15f;
    scenario->mission.cruise_speed_mps = 1.8f;
    scenario->mission.search_yaw_rate_rps = 1.2f;
    scenario->mission.target_confirm_s = 0.3f;
    scenario->mission.target_acquire_timeout_s = 0.6f;
    scenario->mission.terminal_range_m = 1.5f;
    scenario->mission.target_lost_timeout_s = 1.0f;
    scenario->mission.recovery_hold_s = 1.2f;
    scenario->mission.recovery_tilt_ok_rad = 0.30f;
    scenario->mission.breakaway_height_m = 0.5f;
    scenario->mission.home_region_tol_m = 0.4f;
    scenario->mission.home_approach_radius_m = 1.5f;
    scenario->mission.home_search_alt_m = 0.8f;
    scenario->mission.homing_fine_radius_m = 0.30f;
    scenario->mission.homing_yaw_tol_rad = 0.30f;
    scenario->mission.dock_alt_m = 0.08f;
    scenario->mission.dock_lateral_tol_m = 0.08f;
    scenario->mission.dock_capture_tol_m = 0.15f;
    scenario->mission.dock_capture_max_alt_m = 0.25f;
    scenario->mission.dock_land_vel_max = 0.10f;
    scenario->mission.dock_blind_land_alt_m = 0.35f;
    scenario->mission.home_lost_timeout_s = 0.5f;
    scenario->mission.docking_timeout_s = 8.0f;
    scenario->mission.dock_contact_confirm_s = 0.15f;
    scenario->mission.estimator_lost_timeout_s = 3.0f;
    scenario->mission.self_check_s = 0.3f;
    scenario->mission.docked_launch_delay_s = 0.5f;

    scenario->estimator.mode = EST_MODE_INS;
    scenario->estimator.vo_degraded_after_s = 0.6f;
    scenario->estimator.vo_lost_after_s = 2.0f;
    scenario->estimator.recovering_hold_s = 0.5f;
    scenario->estimator.lost_timeout_s = 1.5f;
    scenario->estimator.impact_blind_s = 0.4f;
    scenario->estimator.lost_on_impact = 0u;
    scenario->estimator.kp_tilt = 2.0f;
    scenario->estimator.ki_gyro_bias = 0.05f;
    scenario->estimator.kp_vo_pos = 2.0f;
    scenario->estimator.kp_vo_vel = 3.0f;
    scenario->estimator.kp_vo_yaw = 0.0f;
    scenario->estimator.kp_tof = 2.0f;
    scenario->estimator.kp_tof_vel = 8.0f;

    scenario->impact.accel_spike_threshold = 12.0f;
    scenario->impact.gyro_spike_threshold = 12.0f;
    scenario->impact.confirm_samples = 1u;
    impact_detector_default_fusion_config(&scenario->impact_fusion);
    impact_recovery_default_config(&scenario->recovery);

    scenario->safety.max_mission_time_s = 30.0f;
    scenario->safety.soft_return_deadline_s = 24.0f;
    scenario->safety.hard_return_deadline_s = 30.0f;
    scenario->safety.geofence_radius_m = 12.0f;
    scenario->safety.geofence_min_alt_m = -0.20f;
    scenario->safety.geofence_max_alt_m = 3.0f;
    scenario->safety.min_estimator_quality = 0.20f;
    scenario->safety.controller_saturation_timeout_s = 2.5f;
    scenario->safety.collision_critical_timeout_s = 0.75f;
    scenario->safety.trajectory_invalid_timeout_s = 0.25f;

    scenario->ctrl.kp_pos = 2.0f;
    scenario->ctrl.kp_vel = 3.0f;
    scenario->ctrl.max_vel = 2.0f;
    scenario->ctrl.max_accel = 6.0f;
    scenario->ctrl.kp_yaw = 3.0f;
    scenario->ctrl.max_yaw_rate = 2.0f;
    scenario->dynamics.drag = 0.5f;
    scenario->dynamics.max_speed = 2.5f;
    scenario->dynamics.max_yaw_rate = 3.0f;
    scenario->dynamics.max_att_rate = 10.0f;

    scenario->terminal.approach_speed = 1.2f;
    scenario->terminal.kp = 1.5f;
    scenario->terminal.min_closing_speed = 0.2f;
    scenario->terminal.closing_range_m = 2.4f;
    scenario->terminal.final_align_range_m = 1.0f;
    scenario->terminal.contact_range_m = 0.35f;
    scenario->terminal.closing_speed_mps = 0.9f;
    scenario->terminal.final_speed_mps = 0.55f;
    scenario->terminal.contact_speed_mps = 0.35f;
    scenario->terminal.kp_vertical = 1.2f;
    scenario->terminal.max_vertical_speed_mps = 0.6f;
    scenario->terminal.max_accel_mps2 = 3.0f;
    scenario->terminal.yaw_align_tolerance_rad = 0.35f;
    scenario->terminal.min_confidence = 0.15f;
    scenario->terminal.loss_grace_s = 0.35f;
    scenario->terminal.reacquire_timeout_s = 1.0f;

    scenario->home_guidance.search_alt = 0.8f;
    scenario->home_guidance.descend_speed = 0.45f;
    scenario->home_guidance.kp_lateral = 1.0f;
    scenario->home_guidance.max_lateral_speed = 0.4f;
    scenario->home_guidance.lateral_tol = 0.10f;
    scenario->home_guidance.spiral_rate = 0.25f;
    scenario->home_guidance.spiral_max_radius = 1.5f;
    scenario->home_guidance.spiral_omega = 2.0f;
    scenario->home_guidance.blind_land_alt = 0.35f;
    scenario->home_guidance.blind_land_timeout = 1.0f;
    scenario->home_guidance.kp_yaw = 0.8f;
    scenario->home_guidance.yaw_tolerance_rad = 0.30f;

    trajectory_planner_default_config(&scenario->planner);
    obstacle_avoidance_default_config(&scenario->obstacle_avoidance);
    obstacle_set_init(&scenario->obstacles);
    collision_init(&scenario->swarm_collision, 0.60f);
    swarm_avoidance_default_config(&scenario->swarm_avoidance);

    camera_init(&scenario->cam_forward, 180.0f, 180.0f, 160.0f, 120.0f,
                320.0f, 240.0f, CAM_MOUNT_FORWARD);
    camera_init(&scenario->cam_down, 180.0f, 180.0f, 160.0f, 120.0f,
                320.0f, 240.0f, CAM_MOUNT_DOWN);
    scenario->target_size_m = 0.30f;
    scenario->marker_size_m = 0.25f;
    scenario->use_flow_vo = 1u;
    scenario->flow.min_height = 0.05f;
    scenario->flow.max_height = 5.0f;
    scenario->flow.min_features = 4u;
    scenario->flow.outlier_residual_px = 3.0f;
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
        scenario->other_agent.pos = vec3(1.8f, 1.1f, 1.2f);
        scenario->other_agent.vel = vec3(0.0f, -0.20f, 0.0f);
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
