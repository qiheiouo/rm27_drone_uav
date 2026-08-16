/*
 * main.c - host 仿真主循环（第二阶段）
 *
 * 传感链路（每 tick）：
 *   动力学真值 → IMU(机体系比力+偏置) / VO(漂移+blackout) / 相机(像素观测)
 *   → INS 估计器(融合+健康机) → 撞击检测
 *   → 相机重建(像素+已知尺寸+姿态 → 相对位置) → trackers
 *   → marker 捕获上升沿 → 重定位注入（第三层定位）
 *   → safety → mission_fsm → guidance → pos_controller → 动力学
 *
 * 参数：
 *   --seed N            随机种子（偏置/漂移/噪声，默认 12345）
 *   --lost-on-impact    最坏情况：撞击直接导致估计器 LOST
 *   --truth-mode        对照组：估计器透传 VO（第一阶段行为）
 *
 * 退出码：0 = 任务完成（DOCKED）；2 = 任务失败（紧急降落）；3 = 仿真超时。
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "scenario.h"
#include "sim_sensors.h"
#include "camera.h"
#include "mission_fsm.h"
#include "guidance.h"
#include "agent_state.h"
#include "collision_interface.h"

int main(int argc, char **argv)
{
    Scenario sc;
    scenario_default(&sc);

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--lost-on-impact") == 0) {
            sc.estimator.lost_on_impact = 1u;
        } else if (strcmp(argv[i], "--truth-mode") == 0) {
            sc.estimator.mode = EST_MODE_TRUTH;
        } else if (strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
            sc.seed = (uint32_t)strtoul(argv[++i], 0, 10);
        } else {
            printf("unknown arg: %s\n", argv[i]);
            return 1;
        }
    }

    /* ---- 模块实例（全部静态分配） ---- */
    SimState      sim;
    SimSensors    sensors;
    StateEstimator estimator;
    ImpactDetector impact_det;
    TargetTracker  tracker;
    HomeDetector   home_det;
    SafetyMonitor  safety;
    MissionFsm     fsm;
    SwarmView      swarm;          /* 第一版 other_count = 0 */
    CollisionConfig collision_cfg;

    sim_dynamics_init(&sim, sc.home_pos, 0.0f);
    sim_sensors_init(&sensors, sc.seed);
    if (sc.estimator.mode == EST_MODE_TRUTH) {
        /* 对照组 = 理想真值透传（第一阶段语义）：VO 无漂移无噪声 */
        sensors.vo_drift_vel = vec3_zero();
        sensors.vo_yaw_drift_rate = 0.0f;
        sensors.vo_pos_noise = 0.0f;
        sensors.vo_vel_noise = 0.0f;
        sensors.vo_yaw_noise = 0.0f;
    }
    estimator_init(&estimator, &sc.estimator);
    impact_detector_init(&impact_det, &sc.impact);
    target_tracker_init(&tracker, 5.0f, sc.dt);
    home_detector_init(&home_det, 5.0f, sc.dt);
    safety_init(&safety, &sc.safety);
    mission_fsm_init(&fsm, &sc.mission, sc.home_pos,
                     &sc.outbound_route, &sc.search_route);
    swarm_view_init(&swarm, 1u);
    collision_init(&collision_cfg, 0.6f);

    CtrlOutput ctrl = { {0.0f, 0.0f, 0.0f}, 0.0f };

    float   last_impact_t = -100.0f;
    float   last_relocalize_t = -100.0f;
    float   t = 0.0f;
    int     next_log_centi = 0;
    float   max_pos_err = 0.0f;   /* 估计误差统计（真值对比，仅仿真可观） */

    printf("# RM Drone Nav - Plan B phase-2 closed-loop simulation\n");
    printf("# dt=%.3f budget=%.1fs seed=%u est=%s lost_on_impact=%u\n",
           sc.dt, sc.safety.max_mission_time_s, sc.seed,
           sc.estimator.mode == EST_MODE_INS ? "INS+VO" : "TRUTH",
           sc.estimator.lost_on_impact);

    while (t < sc.sim_max_time_s) {
        uint32_t t_ms = (uint32_t)(t * 1000.0f);

        /* 撞击后短暂视觉冻结（运动模糊/特征骤减） */
        uint8_t vision_freeze = (t - last_impact_t < sc.vision_freeze_s) ? 1u : 0u;

        /* ---- 传感 ---- */
        ImuSample  imu;
        OdomSample vo;
        PixelObs   tpix, hpix;
        sim_sensors_imu(&sensors, &sim, t_ms, &imu);
        sim_sensors_vo(&sensors, &sim, vision_freeze, sc.dt, t_ms, &vo);
        sim_sensors_target(&sensors, &sim, &sc.cam_forward, sc.target_pos,
                           sc.target_size_m, vision_freeze, t_ms, &tpix);
        sim_sensors_home(&sensors, &sim, &sc.cam_down, sc.home_pos,
                         sc.marker_size_m, vision_freeze, t_ms, &hpix);

        /* ---- 估计 / 撞击检测 ---- */
        estimator_update(&estimator, &imu, &vo, sc.dt);

        if (impact_detector_update(&impact_det, &imu)) {
            estimator_notify_impact(&estimator);
            printf("[t=%6.2f] EVENT impact detected (dev=%.1f m/s^2, |w|=%.1f rad/s)\n",
                   t, fabsf(vec3_norm(imu.accel) - NAV_GRAVITY), vec3_norm(imu.gyro));
        }

        /* ---- 相机重建：像素 + 已知尺寸 + 估计姿态 → 相对位置 ---- */
        TargetObs tobs;
        tobs.visible = tpix.visible;
        tobs.timestamp_ms = t_ms;
        tobs.rel_pos = tpix.visible
            ? camera_reconstruct_nav(&sc.cam_forward, tpix.u, tpix.v, tpix.size_px,
                                     sc.target_size_m, estimator.out.att)
            : vec3_zero();

        HomeObs hobs;
        hobs.visible = hpix.visible;
        hobs.timestamp_ms = t_ms;
        hobs.rel_pos = hpix.visible
            ? camera_reconstruct_nav(&sc.cam_down, hpix.u, hpix.v, hpix.size_px,
                                     sc.marker_size_m, estimator.out.att)
            : vec3_zero();

        target_tracker_update(&tracker, &tobs, sc.dt);
        home_detector_update(&home_det, &hobs, sc.dt);

        /* ---- 第三层定位：marker 捕获 → 注入绝对参考，消除 VIO 漂移 ----
         * 冷却 1 s 防止可见性抖动导致反复跳变 */
        if (hobs.visible && t - last_relocalize_t >= 1.0f) {
            NavState abs_ref = estimator.out;
            abs_ref.pos = vec3_sub(sc.home_pos, hobs.rel_pos);
            float correction = vec3_dist(abs_ref.pos, estimator.out.pos);
            if (correction >= 0.05f || estimator.out.status != EST_TRACKING) {
                estimator_notify_relocalized(&estimator, &abs_ref);
                last_relocalize_t = t;
                printf("[t=%6.2f] EVENT relocalized on base marker (drift was %.2f m)\n",
                       t, vec3_dist(estimator.out.pos, sim.pos));
            }
        }

        safety_update(&safety, &estimator.out, &sc.home_pos, sc.dt);

        /* ---- 任务状态机 ---- */
        MissionInput  min;
        MissionOutput mout;
        min.nav = estimator.out;
        min.target = tracker.out;
        min.home = home_det.out;
        min.impact_detected = impact_det.triggered;
        min.start_command = 1u;
        min.time_exceeded = safety.time_exceeded;
        min.geofence_violation = safety.geofence_violation;
        min.dt = sc.dt;

        mission_fsm_update(&fsm, &min, &mout);

        if (mout.state_changed) {
            printf("[t=%6.2f] STATE -> %s (est=%s)\n",
                   t, mission_state_name(mout.state),
                   est_status_name(estimator.out.status));
        }

        /* 开始新一轮目标接近（进入 TARGET_TRACK）时布防撞击检测器；
         * 不清除 TERMINAL 恢复时的锁存——撞击物理上已经发生过 */
        if (mout.state == MS_TARGET_TRACK && mout.state_changed) {
            impact_detector_reset(&impact_det);
        }

        /* ---- 制导 ---- */
        GuidanceOutput gout;
        switch (mout.guidance) {
        case GM_TAKEOFF:
            guidance_takeoff(&mout.hold_pos, &estimator.out, &gout);
            break;
        case GM_HOLD:
            guidance_hold(&mout.hold_pos, &estimator.out, &gout);
            break;
        case GM_WAYPOINT:
            cruise_guidance_update(&mout.current_waypoint, &estimator.out, &gout);
            break;
        case GM_TERMINAL:
            target_guidance_update(&tracker.out, &estimator.out, &sc.terminal, &gout);
            break;
        case GM_RECOVERY:
            recovery_guidance_update(&estimator.out, sc.recovery_damping, &gout);
            break;
        case GM_HOME_SERVO:
            home_guidance_update(&home_det.out, &estimator.out, &sc.home_pos,
                                 &sc.home_guidance, &gout);
            break;
        case GM_LAND:
            guidance_land(&estimator.out, 0.3f, &gout);
            break;
        case GM_NONE:
        default:
            gout.pos_sp = estimator.out.pos;
            gout.vel_sp = vec3_zero();
            gout.yaw_sp = estimator.out.yaw;
            gout.use_pos_sp = 0u;
            break;
        }

        /* ---- 控制 ---- */
        pos_controller_update(&sc.ctrl, &gout, &estimator.out, &ctrl);

        /* ---- 撞击注入：TERMINAL 中接触目标 ----
         * 可重复触发（再次接近会再次接触），0.5 s 不应期防止单帧多注入 */
        SimImpact impact;
        impact.active = 0u;
        impact.delta_v = vec3_zero();
        impact.delta_yaw = 0.0f;
        impact.delta_pitch = 0.0f;
        impact.delta_roll = 0.0f;
        if (mout.state == MS_TERMINAL &&
            t - last_impact_t >= 0.5f &&
            vec3_dist(sim.pos, sc.target_pos) <= sc.impact_range_m) {
            impact.active = 1u;
            impact.delta_v = vec3(-1.8f, 1.2f, 0.9f);   /* 弹开 + 上抛 */
            impact.delta_yaw = 0.9f;
            impact.delta_pitch = 0.5f;
            impact.delta_roll = 0.4f;
            last_impact_t = t;
            printf("[t=%6.2f] EVENT physical contact with target\n", t);
        }

        /* ---- 动力学 ---- */
        sim_dynamics_step(&sim, &ctrl, &sc.dynamics, &impact, sc.dt);

        /* 估计误差统计（真值仅仿真可见，算法不使用） */
        {
            float err = vec3_dist(estimator.out.pos, sim.pos);
            if (estimator.out.status != EST_LOST && err > max_pos_err) {
                max_pos_err = err;
            }
        }

        /* 多机接口占位：第一版无其他智能体 */
        swarm.self.pos = estimator.out.pos;
        swarm.self.vel = estimator.out.vel;
        (void)collision_check(&collision_cfg, 0, 0u, &swarm);

        /* ---- 日志 ---- */
        if ((int)(t * 100.0f) >= next_log_centi) {
            next_log_centi += 100;   /* 每 1 s 一行 */
            Quatf dq_att = quat_mul(quat_conj(estimator.out.att), sim.att);
            if (dq_att.w < 0.0f) { dq_att.w = -dq_att.w; }
            float att_err_deg = 2.0f * acosf(clampf(dq_att.w, -1.0f, 1.0f)) * 57.2958f;
            printf("[t=%6.2f] %-18s pos=(%5.2f,%5.2f,%4.2f) err=%4.2f att_err=%4.1f tgt=%c home=%c truth=(%5.2f,%5.2f,%4.2f)\n",
                   t, mission_state_name(mout.state),
                   estimator.out.pos.x, estimator.out.pos.y, estimator.out.pos.z,
                   vec3_dist(estimator.out.pos, sim.pos), att_err_deg,
                   tracker.out.visible ? 'Y' : 'N',
                   home_det.out.visible ? 'Y' : 'N',
                   sim.pos.x, sim.pos.y, sim.pos.z);
        }

        /* ---- 终止条件 ---- */
        if (mout.mission_complete) {
            float dist_home = vec3_dist(sim.pos, sc.home_pos);
            printf("[t=%6.2f] MISSION_SUCCESS docked, dist_to_home=%.3f m, "
                   "elapsed=%.2f s (budget %.1f s), max_est_err=%.2f m\n",
                   t, dist_home, t, sc.safety.max_mission_time_s, max_pos_err);
            return 0;
        }
        if (mout.mission_failed) {
            printf("[t=%6.2f] MISSION_FAILED emergency landed\n", t);
            return 2;
        }

        t += sc.dt;
    }

    printf("MISSION_FAILED simulation timeout (%.1f s), last state=%s\n",
           t, mission_state_name(fsm.state));
    return 3;
}
