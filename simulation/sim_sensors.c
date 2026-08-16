#include "sim_sensors.h"

/* 确定性 LCG：返回 [-1, 1] 均匀分布 */
static float lcg_uniform(SimSensors *sen)
{
    sen->rng_state = sen->rng_state * 1664525u + 1013904223u;
    return ((float)(sen->rng_state >> 8) / 8388608.0f) - 1.0f;
}

static float lcg_gauss(SimSensors *sen, float sigma)
{
    return (lcg_uniform(sen) + lcg_uniform(sen)) * 0.5f * sigma * 1.7320508f;
}

void sim_sensors_init(SimSensors *sen, uint32_t seed)
{
    sen->rng_state = seed ? seed : 1u;

    /* 由 seed 决定的"本次上电"固定偏置与漂移特性 */
    sen->imu_accel_bias = vec3(lcg_gauss(sen, 0.05f), lcg_gauss(sen, 0.05f), lcg_gauss(sen, 0.05f));
    sen->imu_gyro_bias  = vec3(lcg_gauss(sen, 0.01f), lcg_gauss(sen, 0.01f), lcg_gauss(sen, 0.01f));
    sen->vo_drift_vel   = vec3(lcg_gauss(sen, 0.03f), lcg_gauss(sen, 0.03f), lcg_gauss(sen, 0.01f));
    sen->vo_yaw_drift_rate = lcg_gauss(sen, 0.01f);

    sen->imu_accel_noise = 0.05f;
    sen->imu_gyro_noise = 0.01f;

    sen->vo_drift = vec3_zero();
    sen->vo_yaw_drift = 0.0f;
    sen->vo_pos_noise = 0.03f;
    sen->vo_vel_noise = 0.05f;
    sen->vo_yaw_noise = 0.01f;

    sen->pixel_noise = 1.0f;
    sen->target_min_px = 15.0f;   /* fx=180, 0.3m 目标 → 检测距离上限约 3.6 m */
    sen->home_min_px = 8.0f;
}

void sim_sensors_imu(SimSensors *sen, const SimState *truth, uint32_t t_ms, ImuSample *out)
{
    out->accel = vec3_add(truth->spec_force_body, sen->imu_accel_bias);
    out->accel.x += lcg_gauss(sen, sen->imu_accel_noise);
    out->accel.y += lcg_gauss(sen, sen->imu_accel_noise);
    out->accel.z += lcg_gauss(sen, sen->imu_accel_noise);

    out->gyro = vec3_add(truth->gyro_body, sen->imu_gyro_bias);
    out->gyro.x += lcg_gauss(sen, sen->imu_gyro_noise);
    out->gyro.y += lcg_gauss(sen, sen->imu_gyro_noise);
    out->gyro.z += lcg_gauss(sen, sen->imu_gyro_noise);

    out->timestamp_ms = t_ms;
}

void sim_sensors_vo(SimSensors *sen, const SimState *truth,
                    uint8_t vision_freeze, float dt, uint32_t t_ms, OdomSample *out)
{
    /* 漂移随时间累积 */
    sen->vo_drift = vec3_add(sen->vo_drift, vec3_scale(sen->vo_drift_vel, dt));
    sen->vo_yaw_drift += sen->vo_yaw_drift_rate * dt;

    out->valid = vision_freeze ? 0u : 1u;
    out->timestamp_ms = t_ms;

    out->pos = vec3_add(truth->pos, sen->vo_drift);
    out->pos.x += lcg_gauss(sen, sen->vo_pos_noise);
    out->pos.y += lcg_gauss(sen, sen->vo_pos_noise);
    out->pos.z += lcg_gauss(sen, sen->vo_pos_noise);

    out->vel = vec3(truth->vel.x + lcg_gauss(sen, sen->vo_vel_noise),
                    truth->vel.y + lcg_gauss(sen, sen->vo_vel_noise),
                    truth->vel.z + lcg_gauss(sen, sen->vo_vel_noise));

    out->yaw = wrap_pi(truth->yaw + sen->vo_yaw_drift + lcg_gauss(sen, sen->vo_yaw_noise));
    out->yaw_rate = truth->yaw_rate;
    out->att = truth->att;
}

void sim_sensors_target(SimSensors *sen, const SimState *truth, const CameraModel *cam,
                        Vec3f target_pos, float target_size,
                        uint8_t vision_freeze, uint32_t t_ms, PixelObs *out)
{
    Vec3f rel_nav = vec3_sub(target_pos, truth->pos);
    Vec3f rel_body = quat_rotate_inv(truth->att, rel_nav);
    float u = 0.0f, v = 0.0f, size_px = 0.0f;

    out->timestamp_ms = t_ms;
    out->visible = 0u;
    out->u = out->v = out->size_px = 0.0f;

    if (vision_freeze) {
        return;
    }
    if (camera_project(cam, rel_body, target_size, &u, &v, &size_px) &&
        size_px >= sen->target_min_px) {
        out->visible = 1u;
        out->u = u + lcg_gauss(sen, sen->pixel_noise);
        out->v = v + lcg_gauss(sen, sen->pixel_noise);
        out->size_px = size_px + lcg_gauss(sen, sen->pixel_noise);
    }
}

void sim_sensors_home(SimSensors *sen, const SimState *truth, const CameraModel *cam,
                      Vec3f home_pos, float marker_size,
                      uint8_t vision_freeze, uint32_t t_ms, PixelObs *out)
{
    Vec3f rel_nav = vec3_sub(home_pos, truth->pos);
    Vec3f rel_body = quat_rotate_inv(truth->att, rel_nav);
    float u = 0.0f, v = 0.0f, size_px = 0.0f;

    out->timestamp_ms = t_ms;
    out->visible = 0u;
    out->u = out->v = out->size_px = 0.0f;

    if (vision_freeze) {
        return;
    }
    if (camera_project(cam, rel_body, marker_size, &u, &v, &size_px) &&
        size_px >= sen->home_min_px) {
        out->visible = 1u;
        out->u = u + lcg_gauss(sen, sen->pixel_noise);
        out->v = v + lcg_gauss(sen, sen->pixel_noise);
        out->size_px = size_px + lcg_gauss(sen, sen->pixel_noise);
    }
}
