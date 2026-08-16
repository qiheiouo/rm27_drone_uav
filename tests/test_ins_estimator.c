/*
 * test_ins_estimator.c - INS 估计器单元测试
 *
 * 覆盖：静态悬停零漂、姿态倾斜收敛（Mahony）、VO 位置校正收敛、
 * VO 失效驱动的健康状态机（DEGRADED/LOST/RECOVERING/RELOCALIZED）、
 * 撞击盲期拒绝视觉。
 */
#include <stdio.h>
#include <math.h>
#include "state_estimator.h"

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

static EstimatorConfig test_cfg(void)
{
    EstimatorConfig cfg;
    cfg.mode = EST_MODE_INS;
    cfg.vo_degraded_after_s = 0.6f;
    cfg.vo_lost_after_s = 2.0f;
    cfg.recovering_hold_s = 0.5f;
    cfg.lost_timeout_s = 1.5f;
    cfg.impact_blind_s = 0.4f;
    cfg.lost_on_impact = 0u;
    cfg.kp_tilt = 2.0f;
    cfg.ki_gyro_bias = 0.05f;
    cfg.kp_vo_pos = 2.0f;
    cfg.kp_vo_vel = 3.0f;
    cfg.kp_vo_yaw = 0.0f;
    cfg.kp_tof = 2.0f;
    cfg.kp_tof_vel = 8.0f;
    return cfg;
}

static ImuSample make_imu(Vec3f accel, Vec3f gyro, uint32_t t)
{
    ImuSample s; s.accel = accel; s.gyro = gyro; s.timestamp_ms = t; return s;
}

static OdomSample make_vo(Vec3f pos, Vec3f vel, float yaw, uint8_t valid, uint32_t t)
{
    OdomSample s; s.pos = pos; s.vel = vel; s.yaw = yaw; s.yaw_rate = 0.0f;
    s.att = quat_identity();
    s.valid = valid; s.timestamp_ms = t; return s;
}

int main(void)
{
    const float dt = 0.01f;

    /* ---- 1. 静态悬停：位置不漂 ---- */
    {
        StateEstimator est;
        EstimatorConfig cfg = test_cfg();
        estimator_init(&est, &cfg);
        uint32_t t = 0u;
        for (int i = 0; i < 500; i++) {   /* 5 s */
            ImuSample imu = make_imu(vec3(0.0f, 0.0f, NAV_GRAVITY), vec3_zero(), t);
            OdomSample vo = make_vo(vec3_zero(), vec3_zero(), 0.0f, 1u, t);
            estimator_update(&est, &imu, &vo, -1.0f, dt);
            t += 10u;
        }
        CHECK(vec3_norm(est.out.pos) < 0.05f);
        CHECK(vec3_norm(est.out.vel) < 0.05f);
        CHECK(est.out.status == EST_TRACKING);
        printf("test_ins_estimator: hover PASS\n");
    }

    /* ---- 2. Mahony 姿态收敛：初始姿态有误差，静止时收敛到真实姿态 ---- */
    {
        StateEstimator est;
        EstimatorConfig cfg = test_cfg();
        estimator_init(&est, &cfg);
        /* 真实姿态：绕 x 倾斜 0.2 rad → 机体系比力 = R^T (0,0,g) */
        Quatf q_true = quat_from_axis_angle(vec3(1.0f, 0.0f, 0.0f), 0.2f);
        Vec3f f_body = quat_rotate_inv(q_true, vec3(0.0f, 0.0f, NAV_GRAVITY));
        uint32_t t = 0u;
        for (int i = 0; i < 500; i++) {
            ImuSample imu = make_imu(f_body, vec3_zero(), t);
            OdomSample vo = make_vo(vec3_zero(), vec3_zero(), 0.0f, 1u, t);
            estimator_update(&est, &imu, &vo, -1.0f, dt);
            t += 10u;
        }
        /* 估计姿态作用于实测比力应恢复 (0,0,g) */
        Vec3f f_nav = quat_rotate(est.out.att, f_body);
        CHECK(fabsf(f_nav.z - NAV_GRAVITY) < 0.1f);
        CHECK(fabsf(f_nav.x) < 0.1f && fabsf(f_nav.y) < 0.1f);
        printf("test_ins_estimator: tilt convergence PASS\n");
    }

    /* ---- 3. VO 位置校正：初始位置误差 1 m 应收敛 ---- */
    {
        StateEstimator est;
        EstimatorConfig cfg = test_cfg();
        estimator_init(&est, &cfg);
        est.ins_pos = vec3(1.0f, 0.0f, 0.0f);   /* 注入初始误差 */
        uint32_t t = 0u;
        for (int i = 0; i < 300; i++) {   /* 3 s */
            ImuSample imu = make_imu(vec3(0.0f, 0.0f, NAV_GRAVITY), vec3_zero(), t);
            OdomSample vo = make_vo(vec3_zero(), vec3_zero(), 0.0f, 1u, t);
            estimator_update(&est, &imu, &vo, -1.0f, dt);
            t += 10u;
        }
        CHECK(fabsf(est.out.pos.x) < 0.1f);
        printf("test_ins_estimator: vo correction PASS\n");
    }

    /* ---- 4. 健康机：VO 失效 → DEGRADED → LOST → 恢复 ---- */
    {
        StateEstimator est;
        EstimatorConfig cfg = test_cfg();
        estimator_init(&est, &cfg);
        uint32_t t = 0u;
        ImuSample imu = make_imu(vec3(0.0f, 0.0f, NAV_GRAVITY), vec3_zero(), t);
        OdomSample vo_bad = make_vo(vec3_zero(), vec3_zero(), 0.0f, 0u, t);

        /* 0.7 s VO 无效 → DEGRADED */
        for (int i = 0; i < 70; i++) {
            imu.timestamp_ms = t; vo_bad.timestamp_ms = t;
            estimator_update(&est, &imu, &vo_bad, -1.0f, dt);
            t += 10u;
        }
        CHECK(est.out.status == EST_DEGRADED);

        /* 再继续 1.5 s（累计 2.2 s）→ LOST，输出冻结 */
        for (int i = 0; i < 150; i++) {
            imu.timestamp_ms = t; vo_bad.timestamp_ms = t;
            estimator_update(&est, &imu, &vo_bad, -1.0f, dt);
            t += 10u;
        }
        CHECK(est.out.status == EST_LOST);

        /* VO 恢复 → RECOVERING → RELOCALIZED → TRACKING */
        OdomSample vo_ok = make_vo(vec3_zero(), vec3_zero(), 0.0f, 1u, t);
        int saw_recovering = 0, saw_relocalized = 0;
        for (int i = 0; i < 100; i++) {
            imu.timestamp_ms = t; vo_ok.timestamp_ms = t;
            estimator_update(&est, &imu, &vo_ok, -1.0f, dt);
            t += 10u;
            if (est.out.status == EST_RECOVERING) saw_recovering = 1;
            if (est.out.status == EST_RELOCALIZED) saw_relocalized = 1;
        }
        CHECK(saw_recovering && saw_relocalized);
        CHECK(est.out.status == EST_TRACKING);
        printf("test_ins_estimator: health machine PASS\n");
    }

    /* ---- 5. 撞击盲期：冻结期内 VO 有效也不被采纳 ---- */
    {
        StateEstimator est;
        EstimatorConfig cfg = test_cfg();
        estimator_init(&est, &cfg);
        uint32_t t = 0u;
        ImuSample imu = make_imu(vec3(0.0f, 0.0f, NAV_GRAVITY), vec3_zero(), t);
        OdomSample vo = make_vo(vec3_zero(), vec3_zero(), 0.0f, 1u, t);

        estimator_update(&est, &imu, &vo, -1.0f, dt);
        t += 10u;
        estimator_notify_impact(&est);
        CHECK(est.out.status == EST_DEGRADED);

        /* 盲期内持续喂有效 VO：不应进入 RECOVERING/TRACKING */
        for (int i = 0; i < 30; i++) {   /* 0.3 s < 0.4 s 盲期 */
            imu.timestamp_ms = t; vo.timestamp_ms = t;
            estimator_update(&est, &imu, &vo, -1.0f, dt);
            t += 10u;
            CHECK(est.out.status == EST_DEGRADED);
        }
        /* 盲期结束后 VO 被采纳 → RECOVERING → ... → TRACKING */
        for (int i = 0; i < 200; i++) {
            imu.timestamp_ms = t; vo.timestamp_ms = t;
            estimator_update(&est, &imu, &vo, -1.0f, dt);
            t += 10u;
        }
        CHECK(est.out.status == EST_TRACKING);
        printf("test_ins_estimator: impact blind PASS\n");
    }

    /* ---- 6. ToF 高度融合：z 初始误差 1 m 应收敛 ---- */
    {
        StateEstimator est;
        EstimatorConfig cfg = test_cfg();
        cfg.kp_tof = 2.0f;
        estimator_init(&est, &cfg);
        est.ins_pos = vec3(0.0f, 0.0f, 1.0f);   /* 注入 z 误差（真值高度 0） */
        uint32_t t = 0u;
        for (int i = 0; i < 300; i++) {   /* 3 s */
            ImuSample imu = make_imu(vec3(0.0f, 0.0f, NAV_GRAVITY), vec3_zero(), t);
            OdomSample vo = make_vo(vec3_zero(), vec3_zero(), 0.0f, 1u, t);
            estimator_update(&est, &imu, &vo, 0.0f, dt);   /* tof 报高度 0 */
            t += 10u;
        }
        CHECK(fabsf(est.out.pos.z) < 0.15f);
        printf("test_ins_estimator: tof height fusion PASS\n");
    }

    if (failures == 0) {
        printf("test_ins_estimator: PASS\n");
        return 0;
    }
    printf("test_ins_estimator: %d FAILURES\n", failures);
    return 1;
}
