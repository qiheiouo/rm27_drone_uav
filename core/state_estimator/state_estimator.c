#include "state_estimator.h"

static float valid_dt(float dt)
{
    return (nav_isfinite(dt) && dt >= 0.0005f && dt <= 0.05f) ? dt : 0.0f;
}

static void update_output_health(StateEstimator *est)
{
    est->out.mode = est->cfg.mode;
    switch (est->out.status) {
    case EST_TRACKING:
    case EST_RELOCALIZED:
        est->out.validity = NAV_VALID_VALID;
        est->out.quality = clampf(1.0f - 0.25f * est->vo_invalid_time, 0.75f, 1.0f);
        break;
    case EST_VISUAL_AIDED:
    case EST_IMU_ONLY:
    case EST_DEGRADED:
    case EST_RECOVERING:
        est->out.validity = NAV_VALID_DEGRADED;
        est->out.quality = clampf(0.7f - 0.2f * est->vo_invalid_time, 0.2f, 0.75f);
        break;
    case EST_INIT:
    case EST_LOST:
    default:
        est->out.validity = NAV_VALID_INVALID;
        est->out.quality = 0.0f;
        break;
    }
}

void estimator_init(StateEstimator *est, const EstimatorConfig *cfg)
{
    est->cfg = *cfg;
    est->out.pos = vec3_zero();
    est->out.vel = vec3_zero();
    est->out.att = quat_identity();
    est->out.angular_velocity = vec3_zero();
    est->out.linear_acceleration = vec3_zero();
    est->out.yaw = 0.0f;
    est->out.yaw_rate = 0.0f;
    est->out.timestamp_ms = 0u;
    est->out.validity = NAV_VALID_INVALID;
    est->out.quality = 0.0f;
    est->out.mode = cfg->mode;
    est->out.status = EST_INIT;
    est->timer = 0.0f;
    est->vo_invalid_time = 0.0f;
    est->blind_time = 0.0f;
    est->last_valid = est->out;
    est->ins_pos = vec3_zero();
    est->ins_vel = vec3_zero();
    est->ins_att = quat_identity();
    est->gyro_bias = vec3_zero();
    est->accel_bias = vec3_zero();
    est->good_visual_frames = 0u;
    est->rejected_measurements = 0u;
    est->max_position_innovation_m = 4.0f;
    est->max_velocity_innovation_mps = 5.0f;
}

void estimator_notify_impact(StateEstimator *est)
{
    est->blind_time = est->cfg.impact_blind_s;
    est->good_visual_frames = 0u;
    if (est->cfg.lost_on_impact) {
        est->out.status = EST_LOST;
        est->last_valid = est->out;
    } else {
        est->out.status = EST_DEGRADED;
    }
    est->timer = 0.0f;
    update_output_health(est);
}

void estimator_notify_relocalized(StateEstimator *est, const NavState *absolute_ref)
{
    est->ins_pos = absolute_ref->pos;
    est->ins_vel = absolute_ref->vel;
    est->ins_att = quat_normalize(absolute_ref->att);
    est->out = *absolute_ref;
    est->out.att = est->ins_att;
    est->out.status = EST_RELOCALIZED;
    est->timer = 0.0f;
    est->vo_invalid_time = 0.0f;
    est->good_visual_frames = 0u;
    update_output_health(est);
}

uint8_t estimator_apply_visual_correction(StateEstimator *est,
                                          const VisualCorrection *correction,
                                          float dt)
{
    float step = valid_dt(dt);
    Vec3f pos_error;
    Vec3f vel_error;
    float pos_gate;
    float vel_gate;

    if (!correction->valid || step <= 0.0f ||
        !vec3_is_finite(correction->position) ||
        !vec3_is_finite(correction->velocity)) {
        return 0u;
    }

    pos_error = vec3_sub(correction->position, est->ins_pos);
    vel_error = vec3_sub(correction->velocity, est->ins_vel);
    pos_gate = correction->max_position_innovation_m > 0.0f
        ? correction->max_position_innovation_m : est->max_position_innovation_m;
    vel_gate = correction->max_velocity_innovation_mps > 0.0f
        ? correction->max_velocity_innovation_mps : est->max_velocity_innovation_mps;

    if (vec3_norm_xy(pos_error) > pos_gate || vec3_norm_xy(vel_error) > vel_gate) {
        if (est->rejected_measurements < 65535u) {
            est->rejected_measurements++;
        }
        return 0u;
    }

    est->ins_pos.x += clampf(correction->position_gain * step, 0.0f, 1.0f) * pos_error.x;
    est->ins_pos.y += clampf(correction->position_gain * step, 0.0f, 1.0f) * pos_error.y;
    est->ins_vel.x += clampf(correction->velocity_gain * step, 0.0f, 1.0f) * vel_error.x;
    est->ins_vel.y += clampf(correction->velocity_gain * step, 0.0f, 1.0f) * vel_error.y;
    return 1u;
}

static void ins_predict(StateEstimator *est, const ImuSample *imu, float dt)
{
    Vec3f f_body = vec3_sub(imu->accel, est->accel_bias);
    Vec3f gyro = vec3_sub(imu->gyro, est->gyro_bias);
    float fnorm = vec3_norm(f_body);
    Vec3f a_nav;

    if (fabsf(fnorm - NAV_GRAVITY) < 1.5f && fnorm > 1e-3f) {
        Vec3f up_meas = vec3_scale(f_body, 1.0f / fnorm);
        Vec3f up_est = quat_rotate_inv(est->ins_att, vec3(0.0f, 0.0f, 1.0f));
        Vec3f err = vec3_cross(up_meas, up_est);
        gyro = vec3_add(gyro, vec3_scale(err, est->cfg.kp_tilt));
        est->gyro_bias = vec3_sub(est->gyro_bias,
                                  vec3_scale(err, est->cfg.ki_gyro_bias * dt));
        est->gyro_bias = vec3_clamp_norm(est->gyro_bias, 0.5f);
    }

    est->ins_att = quat_integrate_gyro(est->ins_att, gyro, dt);
    a_nav = vec3_add(quat_rotate(est->ins_att, f_body),
                     vec3(0.0f, 0.0f, -NAV_GRAVITY));
    est->ins_pos = vec3_add(est->ins_pos,
                            vec3_add(vec3_scale(est->ins_vel, dt),
                                     vec3_scale(a_nav, 0.5f * dt * dt)));
    est->ins_vel = vec3_add(est->ins_vel, vec3_scale(a_nav, dt));
    est->out.angular_velocity = gyro;
    est->out.linear_acceleration = a_nav;
}

static uint8_t ins_correct_vo(StateEstimator *est, const OdomSample *vo, float dt)
{
    VisualCorrection correction;
    correction.valid = vo->valid;
    correction.position = vo->pos;
    correction.velocity = vo->vel;
    correction.position_gain = est->cfg.kp_vo_pos;
    correction.velocity_gain = est->cfg.kp_vo_vel;
    correction.max_position_innovation_m = est->max_position_innovation_m;
    correction.max_velocity_innovation_mps = est->max_velocity_innovation_mps;
    correction.timestamp_ms = vo->timestamp_ms;
    return estimator_apply_visual_correction(est, &correction, dt);
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

static void health_update(StateEstimator *est, uint8_t vo_valid, float dt)
{
    est->timer += dt;
    if (est->blind_time > 0.0f) {
        est->blind_time -= dt;
        if (est->blind_time < 0.0f) {
            est->blind_time = 0.0f;
        }
        vo_valid = 0u;
    }

    if (vo_valid) {
        est->vo_invalid_time = 0.0f;
        if (est->good_visual_frames < 65535u) {
            est->good_visual_frames++;
        }
    } else {
        est->vo_invalid_time += dt;
        est->good_visual_frames = 0u;
    }

    switch (est->out.status) {
    case EST_INIT:
        est->out.status = vo_valid ? EST_VISUAL_AIDED : EST_IMU_ONLY;
        est->timer = 0.0f;
        break;
    case EST_IMU_ONLY:
        if (vo_valid) {
            est->out.status = EST_VISUAL_AIDED;
            est->timer = 0.0f;
        } else if (est->vo_invalid_time >= est->cfg.vo_degraded_after_s) {
            est->out.status = EST_DEGRADED;
            est->timer = 0.0f;
        }
        break;
    case EST_VISUAL_AIDED:
        if (!vo_valid) {
            est->out.status = EST_IMU_ONLY;
            est->timer = 0.0f;
        } else if (est->good_visual_frames >= 5u) {
            est->out.status = EST_TRACKING;
            est->timer = 0.0f;
        }
        break;
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
            est->last_valid.status = EST_LOST;
            est->timer = 0.0f;
        }
        break;
    case EST_LOST:
        if (vo_valid) {
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
            est->last_valid.status = EST_LOST;
            est->timer = 0.0f;
        }
        break;
    case EST_RELOCALIZED:
        est->out.status = EST_TRACKING;
        est->timer = 0.0f;
        break;
    default:
        est->out.status = EST_LOST;
        est->timer = 0.0f;
        break;
    }
    update_output_health(est);
}

void estimator_update(StateEstimator *est, const ImuSample *imu,
                      const OdomSample *vo, float tof_height, float dt)
{
    float step = valid_dt(dt);
    uint8_t vo_valid;
    uint8_t was_lost;

    if (step <= 0.0f || !vec3_is_finite(imu->accel) || !vec3_is_finite(imu->gyro)) {
        est->out.status = EST_DEGRADED;
        update_output_health(est);
        return;
    }

    vo_valid = (vo->valid && est->blind_time <= 0.0f &&
                vec3_is_finite(vo->pos) && vec3_is_finite(vo->vel)) ? 1u : 0u;

    if (est->cfg.mode == EST_MODE_INS) {
        ins_predict(est, imu, step);
        if (vo_valid && est->out.status != EST_LOST) {
            vo_valid = ins_correct_vo(est, vo, step);
        }
        if (nav_isfinite(tof_height) && tof_height >= 0.0f && est->out.status != EST_LOST) {
            float cos_tilt = quat_rotate(est->ins_att, vec3(0.0f, 0.0f, 1.0f)).z;
            if (cos_tilt > 0.5f) {
                float error_z = tof_height * cos_tilt - est->ins_pos.z;
                if (fabsf(error_z) <= 1.5f) {
                    est->ins_pos.z += est->cfg.kp_tof * error_z * step;
                    est->ins_vel.z += est->cfg.kp_tof_vel * error_z * step;
                }
            }
        }
        publish_ins(est, imu);
    } else if (vo_valid) {
        est->out.pos = vo->pos;
        est->out.vel = vo->vel;
        est->out.yaw = vo->yaw;
        est->out.yaw_rate = vo->yaw_rate;
        est->out.att = quat_normalize(vo->att);
        est->out.angular_velocity = vec3(0.0f, 0.0f, vo->yaw_rate);
        est->out.linear_acceleration = vec3_zero();
        est->out.timestamp_ms = vo->timestamp_ms;
        est->ins_pos = est->out.pos;
        est->ins_vel = est->out.vel;
        est->ins_att = est->out.att;
    }

    was_lost = (est->out.status == EST_LOST) ? 1u : 0u;
    health_update(est, vo_valid, step);

    if (est->out.status == EST_LOST) {
        if (!was_lost) {
            est->last_valid = est->out;
            est->last_valid.status = EST_LOST;
        }
        est->out.pos = est->last_valid.pos;
        est->out.vel = vec3_zero();
        est->out.att = est->last_valid.att;
        est->out.yaw = est->last_valid.yaw;
        est->out.status = EST_LOST;
        update_output_health(est);
    }
}

const char *est_status_name(EstStatus status)
{
    switch (status) {
    case EST_INIT:         return "INIT";
    case EST_IMU_ONLY:     return "IMU_ONLY";
    case EST_VISUAL_AIDED: return "VISUAL_AIDED";
    case EST_TRACKING:     return "TRACKING";
    case EST_DEGRADED:     return "DEGRADED";
    case EST_LOST:         return "LOST";
    case EST_RECOVERING:   return "RECOVERING";
    case EST_RELOCALIZED:  return "RELOCALIZED";
    default:               return "?";
    }
}
