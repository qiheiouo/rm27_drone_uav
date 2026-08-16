#include "scenario.h"

void scenario_default(Scenario *sc)
{
    /* ---- 场景 ---- */
    sc->home_pos = vec3(0.0f, 0.0f, 0.0f);
    sc->target_pos = vec3(4.0f, 2.0f, 1.0f);

    wq_init(&sc->outbound_route);
    wq_push(&sc->outbound_route, vec3(2.0f, 0.0f, 1.2f), 1.8f);
    wq_push(&sc->outbound_route, vec3(3.5f, 0.5f, 1.2f), 1.8f);

    /* 搜索扫掠（蛇形），路径经过目标附近保证可检测 */
    wq_init(&sc->search_route);
    wq_push(&sc->search_route, vec3(3.5f, 1.5f, 1.2f), 1.2f);
    wq_push(&sc->search_route, vec3(4.5f, 1.5f, 1.2f), 1.2f);
    wq_push(&sc->search_route, vec3(4.5f, 2.5f, 1.2f), 1.2f);
    wq_push(&sc->search_route, vec3(3.5f, 2.5f, 1.2f), 1.2f);

    /* ---- 任务状态机 ---- */
    sc->mission.takeoff_alt_m = 1.2f;
    sc->mission.waypoint_tol_m = 0.15f;
    sc->mission.cruise_speed_mps = 1.8f;
    sc->mission.target_confirm_s = 0.3f;
    sc->mission.terminal_range_m = 1.5f;
    sc->mission.target_lost_timeout_s = 1.0f;
    sc->mission.recovery_hold_s = 1.2f;
    sc->mission.breakaway_height_m = 0.5f;
    sc->mission.home_region_tol_m = 0.4f;
    sc->mission.home_search_alt_m = 0.8f;
    sc->mission.dock_alt_m = 0.08f;
    sc->mission.dock_lateral_tol_m = 0.08f;
    sc->mission.dock_capture_tol_m = 0.15f;
    sc->mission.dock_capture_max_alt_m = 0.25f;
    sc->mission.dock_land_vel_max = 0.08f;
    sc->mission.dock_blind_land_alt_m = 0.35f;
    sc->mission.home_lost_timeout_s = 0.5f;
    sc->mission.estimator_lost_timeout_s = 3.0f;
    sc->mission.self_check_s = 0.3f;
    sc->mission.docked_launch_delay_s = 0.5f;

    /* ---- 估计器（第二阶段：INS + VO 融合） ---- */
    sc->estimator.mode = EST_MODE_INS;
    sc->estimator.vo_degraded_after_s = 0.6f;
    sc->estimator.vo_lost_after_s = 2.0f;
    sc->estimator.recovering_hold_s = 0.5f;
    sc->estimator.lost_timeout_s = 1.5f;
    sc->estimator.impact_blind_s = 0.4f;
    sc->estimator.lost_on_impact = 0u;      /* 默认退化不丢失；置 1 演练最坏情况 */
    sc->estimator.kp_tilt = 2.0f;
    sc->estimator.ki_gyro_bias = 0.05f;
    /* VO 互补校正增益 (1/s)。
     * kp_vo_yaw = 0：光流 VO 的偏航与 INS 偏航同源（同一陀螺积分），
     * 校正只会把撞击/冻结期间 vf 偏航的瞬时错误灌回估计器（正反馈）。
     * 偏航漂移由重定位时的 vf_set_pose 对齐来兜底。 */
    sc->estimator.kp_vo_pos = 2.0f;
    sc->estimator.kp_vo_vel = 3.0f;
    sc->estimator.kp_vo_yaw = 0.0f;

    /* ---- 撞击检测（比力偏离 1g 判定） ---- */
    sc->impact.accel_spike_threshold = 12.0f;  /* m/s^2 偏离 1g；机动 <7，撞击 >100 */
    sc->impact.gyro_spike_threshold = 6.0f;    /* rad/s */
    sc->impact.confirm_samples = 1u;

    /* ---- 安全 ---- */
    sc->safety.max_mission_time_s = 30.0f;     /* 规则：单次飞行 <= 30 s */
    sc->safety.geofence_radius_m = 12.0f;
    sc->safety.geofence_max_alt_m = 3.0f;

    /* ---- 控制器 ---- */
    sc->ctrl.kp_pos = 2.0f;
    sc->ctrl.kp_vel = 3.0f;
    sc->ctrl.max_vel = 2.0f;
    sc->ctrl.max_accel = 6.0f;
    sc->ctrl.kp_yaw = 3.0f;
    sc->ctrl.max_yaw_rate = 2.0f;

    /* ---- 动力学 ---- */
    sc->dynamics.drag = 0.5f;
    sc->dynamics.max_speed = 2.5f;
    sc->dynamics.max_yaw_rate = 3.0f;
    sc->dynamics.max_att_rate = 5.0f;

    /* ---- 末端制导 ---- */
    sc->terminal.approach_speed = 1.2f;
    sc->terminal.kp = 1.5f;
    sc->terminal.min_closing_speed = 0.3f;

    /* ---- 基座制导 ---- */
    sc->home_guidance.search_alt = 0.8f;
    sc->home_guidance.descend_speed = 0.3f;
    sc->home_guidance.kp_lateral = 1.5f;
    sc->home_guidance.max_lateral_speed = 0.5f;
    sc->home_guidance.lateral_tol = 0.10f;
    sc->home_guidance.spiral_rate = 0.25f;
    sc->home_guidance.spiral_max_radius = 1.5f;
    sc->home_guidance.spiral_omega = 2.0f;
    sc->home_guidance.blind_land_alt = 0.35f;
    sc->home_guidance.blind_land_timeout = 1.0f;

    sc->recovery_damping = 0.8f;

    /* ---- 相机 ---- */
    camera_init(&sc->cam_forward, 180.0f, 180.0f, 160.0f, 120.0f,
                320.0f, 240.0f, CAM_MOUNT_FORWARD);
    camera_init(&sc->cam_down, 180.0f, 180.0f, 160.0f, 120.0f,
                320.0f, 240.0f, CAM_MOUNT_DOWN);
    sc->target_size_m = 0.30f;
    sc->marker_size_m = 0.25f;

    /* ---- VO 前端（光流里程计） ---- */
    sc->use_flow_vo = 1u;
    sc->flow.min_height = 0.05f;
    sc->flow.max_height = 5.0f;
    sc->flow.min_features = 4u;
    sc->flow.outlier_residual_px = 3.0f;
    sc->feature_area_m = 14.0f;
    sc->feature_count = 4000u;   /* ~20 点/m²：低空（停靠段）也能保持 >=4 个可见 */

    /* ---- 仿真控制 ---- */
    sc->dt = 0.01f;                  /* 100 Hz 导航/控制环 */
    sc->sim_max_time_s = 60.0f;      /* 仿真上限（任务预算 30 s 由 safety 保证） */
    sc->impact_range_m = 0.15f;
    sc->vision_freeze_s = 0.4f;
    sc->seed = 12345u;
}
