#include "sim_dynamics.h"

void sim_dynamics_init(SimState *st, Vec3f pos0, float yaw0)
{
    st->pos = pos0;
    st->vel = vec3_zero();
    st->yaw = yaw0;
    st->yaw_rate = 0.0f;
    st->last_accel = vec3_zero();
    st->att = quat_from_axis_angle(vec3(0.0f, 0.0f, 1.0f), yaw0);
    st->att_prev = st->att;
    st->gyro_body = vec3_zero();
    st->spec_force_body = vec3(0.0f, 0.0f, NAV_GRAVITY);
}

void sim_dynamics_step(SimState *st, const CtrlOutput *ctrl,
                       const SimParams *params, const SimImpact *impact, float dt)
{
    Vec3f vel_prev = st->vel;

    /* ---- 平移：控制加速度 + 线性阻尼 ---- */
    Vec3f accel = vec3_sub(ctrl->accel_cmd, vec3_scale(st->vel, params->drag));
    st->vel = vec3_add(st->vel, vec3_scale(accel, dt));

    /* 撞击：瞬时速度/偏航扰动（等效一次大脉冲） */
    if (impact->active) {
        st->vel = vec3_add(st->vel, impact->delta_v);
        st->yaw = wrap_pi(st->yaw + impact->delta_yaw);
    }

    st->vel = vec3_clamp_norm(st->vel, params->max_speed);
    st->pos = vec3_add(st->pos, vec3_scale(st->vel, dt));

    /* 地面约束 */
    if (st->pos.z < 0.0f) {
        st->pos.z = 0.0f;
        if (st->vel.z < 0.0f) {
            st->vel.z = 0.0f;
        }
    }

    float yaw_rate_cmd = clampf(ctrl->yaw_rate_cmd, -params->max_yaw_rate, params->max_yaw_rate);
    st->yaw = wrap_pi(st->yaw + yaw_rate_cmd * dt);
    st->yaw_rate = yaw_rate_cmd;

    /* 实际惯性加速度（含撞击脉冲） */
    st->last_accel = vec3_scale(vec3_sub(st->vel, vel_prev), 1.0f / dt);

    /* ---- 姿态：推力矢量模型 + 姿态环带宽限制 ----
     * 比力（导航系）= 惯性加速度 - 重力 = a + (0,0,g)；
     * 机体 z 轴对齐比力方向，偏航取指令跟踪值。
     * 姿态以有限角速度向目标姿态转动（避免运动学模型产生瞬时陀螺尖峰）。 */
    Vec3f s_nav = vec3_add(st->last_accel, vec3(0.0f, 0.0f, NAV_GRAVITY));
    Quatf q_des = quat_from_zdir_yaw(s_nav, st->yaw);
    Quatf q;
    {
        Quatf dq = quat_mul(quat_conj(st->att), q_des);
        if (dq.w < 0.0f) { dq.x = -dq.x; dq.y = -dq.y; dq.z = -dq.z; dq.w = -dq.w; }
        float ang = 2.0f * acosf(clampf(dq.w, -1.0f, 1.0f));
        float max_ang = params->max_att_rate * dt;
        if (ang > max_ang && ang > 1e-9f) {
            Quatf step = quat_from_axis_angle(vec3(dq.x, dq.y, dq.z), max_ang);
            q = quat_normalize(quat_mul(st->att, step));
        } else {
            q = q_des;
        }
    }

    /* 撞击附加倾角扰动（绕过带宽限制，产生真实陀螺尖峰） */
    if (impact->active) {
        Quatf kick = quat_mul(quat_from_axis_angle(vec3(1.0f, 0.0f, 0.0f), impact->delta_roll),
                              quat_from_axis_angle(vec3(0.0f, 1.0f, 0.0f), impact->delta_pitch));
        q = quat_normalize(quat_mul(q, kick));
    }

    /* 机体系角速度真值：dq = q_prev* ⊗ q，omega ≈ 2*dq.xyz/dt */
    {
        Quatf dq = quat_mul(quat_conj(st->att_prev), q);
        if (dq.w < 0.0f) { dq.x = -dq.x; dq.y = -dq.y; dq.z = -dq.z; }
        st->gyro_body = vec3_scale(vec3(dq.x, dq.y, dq.z), 2.0f / dt);
    }

    st->att = q;
    st->att_prev = q;
    st->spec_force_body = quat_rotate_inv(q, s_nav);
}
