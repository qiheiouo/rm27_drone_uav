#include "state_estimator.h"

void estimator_init(StateEstimator *est, const EstimatorConfig *cfg)
{
    est->cfg = *cfg;
    est->out.pos = vec3_zero();
    est->out.vel = vec3_zero();
    est->out.yaw = 0.0f;
    est->out.yaw_rate = 0.0f;
    est->out.att = quat_identity();
    est->out.status = EST_TRACKING;
    est->out.timestamp_ms = 0u;
    est->timer = 0.0f;
    est->vo_invalid_time = 0.0f;
    est->blind_time = 0.0f;
    est->last_valid = est->out;
    est->ins_pos = vec3_zero();
    est->ins_vel = vec3_zero();
    est->ins_att = quat_identity();
    est->gyro_bias = vec3_zero();
    est->accel_bias = vec3_zero();
}

void estimator_notify_impact(StateEstimator *est)
{
    est->blind_time = est->cfg.impact_blind_s;
    if (est->cfg.lost_on_impact) {
        est->out.status = EST_LOST;
        est->last_valid = est->out;
    } else {
        est->out.status = EST_DEGRADED;
    }
    est->timer = 0.0f;
}

void estimator_notify_relocalized(StateEstimator *est, const NavState *absolute_ref)
{
    est->ins_pos = absolute_ref->pos;
    est->ins_vel = absolute_ref->vel;
    est->ins_att = absolute_ref->att;
    est->out = *absolute_ref;
    est->out.status = EST_RELOCALIZED;
    est->timer = 0.0f;
    est->vo_invalid_time = 0.0f;
}

/* ---------------- INS 预测 ---------------- */

static void ins_predict(StateEstimator *est, const ImuSample *imu, float dt)
{
    Vec3f f_body = vec3_sub(imu->accel, est->accel_bias);
    Vec3f gyro = vec3_sub(imu->gyro, est->gyro_bias);

    /* Mahony：加速度计倾斜修正（仅在大致平稳、比力接近 1g 时启用，
     * 撞击/大机动时自动闭锁，避免把脉冲当重力方向） */
    float fnorm = vec3_norm(f_body);
    if (fabsf(fnorm - NAV_GRAVITY) < 1.5f && fnorm > 1e-3f) {
        Vec3f up_meas = vec3_scale(f_body, 1.0f / fnorm);         /* 机体系实测"上" */
        Vec3f up_est = quat_rotate_inv(est->ins_att, vec3(0.0f, 0.0f, 1.0f));
        Vec3f err = vec3(up_meas.y * up_est.z - up_meas.z * up_est.y,
                         up_meas.z * up_est.x - up_meas.x * up_est.z,
                         up_meas.x * up_est.y - up_meas.y * up_est.x);
        gyro = vec3_add(gyro, vec3_scale(err, est->cfg.kp_tilt));
        est->gyro_bias = vec3_sub(est->gyro_bias, vec3_scale(err, est->cfg.ki_gyro_bias * dt));
    }

    est->ins_att = quat_integrate_gyro(est->ins_att, gyro, dt);

    /* 比力 → 导航系加速度，积分位置速度 */
    Vec3f a_nav = vec3_add(quat_rotate(est->ins_att, f_body),
                           vec3(0.0f, 0.0f, -NAV_GRAVITY));
    est->ins_pos = vec3_add(est->ins_pos,
                            vec3_add(vec3_scale(est->ins_vel, dt),
                                     vec3_scale(a_nav, 0.5f * dt * dt)));
    est->ins_vel = vec3_add(est->ins_vel, vec3_scale(a_nav, dt));
}

/* VO 互补校正（仅水平通道：光流 Vz 可观性弱，z 由 ToF 锚定） */
static void ins_correct_vo(StateEstimator *est, const OdomSample *vo, float dt)
{
    float kp_pos = est->cfg.kp_vo_pos * dt;
    float kp_vel = est->cfg.kp_vo_vel * dt;

    est->ins_pos.x += kp_pos * (vo->pos.x - est->ins_pos.x);
    est->ins_pos.y += kp_pos * (vo->pos.y - est->ins_pos.y);
    est->ins_vel.x += kp_vel * (vo->vel.x - est->ins_vel.x);
    est->ins_vel.y += kp_vel * (vo->vel.y - est->ins_vel.y);
    /* vo->yaw 不校正（与 INS 偏航同源，见 scenario 配置说明） */
    (void)vo;
}

static void publish_ins(StateEstimator *est, const ImuSample *imu)
{
    est->out.pos = est->ins_pos;
    est->out.vel = est->ins_vel;
    est->out.att = est->ins_att;
    est->out.yaw = quat_to_yaw(est->ins_att);
    est->out.yaw_rate = imu->gyro.z - est->gyro_bias.z;
    est->out.timestamp_ms = imu->timestamp_ms;
}

/* ---------------- 健康状态机 ---------------- */

static void health_update(StateEstimator *est, uint8_t vo_valid, float dt)
{
    est->timer += dt;
    if (est->blind_time > 0.0f) {
        est->blind_time -= dt;
        vo_valid = 0u;   /* 撞击后视觉冻结期拒绝视觉输入 */
    }
    if (vo_valid) {
        est->vo_invalid_time = 0.0f;
    } else {
        est->vo_invalid_time += dt;
    }

    switch (est->out.status) {
    case EST_TRACKING:
        if (est->vo_invalid_time >= est->cfg.vo_degraded_after_s) {
            est->out.status = EST_DEGRADED;
            est->timer = 0.0f;
        }
        break;

    case EST_DEGRADED:
        if (vo_valid) {
            est->out.status = EST_RECOVERING;
            est->timer = 0.0f;
        } else if (est->vo_invalid_time >= est->cfg.vo_lost_after_s) {
            est->out.status = EST_LOST;
            est->last_valid = est->out;
            est->timer = 0.0f;
        }
        break;

    case EST_LOST:
        if (vo_valid) {
            est->out.status = EST_RECOVERING;
            est->timer = 0.0f;
        } else if (est->timer >= est->cfg.lost_timeout_s) {
            /* 盲等超时：尝试靠纯 IMU 推算恢复输出（漂移大，但优于冻结） */
            est->out.status = EST_RECOVERING;
            est->timer = 0.0f;
        }
        break;

    case EST_RECOVERING:
        if (vo_valid && est->timer >= est->cfg.recovering_hold_s) {
            est->out.status = EST_RELOCALIZED;
            est->timer = 0.0f;
        } else if (!vo_valid && est->vo_invalid_time >= est->cfg.vo_lost_after_s) {
            est->out.status = EST_LOST;
            est->last_valid = est->out;
            est->timer = 0.0f;
        }
        break;

    case EST_RELOCALIZED:
        est->out.status = EST_TRACKING;
        est->timer = 0.0f;
        break;

    default:
        est->out.status = EST_LOST;
        est->last_valid = est->out;
        est->timer = 0.0f;
        break;
    }
}

void estimator_update(StateEstimator *est, const ImuSample *imu,
                      const OdomSample *vo, float tof_height, float dt)
{
    uint8_t vo_valid = (vo->valid && est->blind_time <= 0.0f) ? 1u : 0u;

    /* ---- 状态预测 ---- */
    if (est->cfg.mode == EST_MODE_INS) {
        ins_predict(est, imu, dt);
        if (vo_valid && est->out.status != EST_LOST) {
            ins_correct_vo(est, vo, dt);
        }
        /* ToF 高度融合：把斜距按姿态投影回垂直高度，锚定 z 通道 */
        if (tof_height >= 0.0f && est->out.status != EST_LOST) {
            float cos_tilt = quat_rotate(est->ins_att, vec3(0.0f, 0.0f, 1.0f)).z;
            if (cos_tilt > 0.5f) {
                float h_meas = tof_height * cos_tilt;
                float err_z = h_meas - est->ins_pos.z;
                est->ins_pos.z += est->cfg.kp_tof * err_z * dt;
                est->ins_vel.z += est->cfg.kp_tof_vel * err_z * dt;
            }
        }
        publish_ins(est, imu);
    } else {
        /* TRUTH 模式：透传里程计 */
        if (vo->valid) {
            est->out.pos = vo->pos;
            est->out.vel = vo->vel;
            est->out.yaw = vo->yaw;
            est->out.yaw_rate = vo->yaw_rate;
            est->out.att = vo->att;
            est->out.timestamp_ms = vo->timestamp_ms;
        }
        est->ins_pos = est->out.pos;
        est->ins_vel = est->out.vel;
    }

    /* ---- LOST 时冻结输出 ---- */
    uint8_t was_lost = (est->out.status == EST_LOST) ? 1u : 0u;

    health_update(est, vo->valid, dt);

    if (est->out.status == EST_LOST) {
        if (!was_lost) {
            est->last_valid = est->out;
        }
        est->out = est->last_valid;
    }
}

const char *est_status_name(EstStatus s)
{
    switch (s) {
    case EST_TRACKING:    return "TRACKING";
    case EST_DEGRADED:    return "DEGRADED";
    case EST_LOST:        return "LOST";
    case EST_RECOVERING:  return "RECOVERING";
    case EST_RELOCALIZED: return "RELOCALIZED";
    default:              return "?";
    }
}
