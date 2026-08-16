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

    /* ---- 姿态：推力矢量模型 + 姿态环带宽限制 ----
     * 期望比力 s_cmd = 指令加速度 + 重力补偿；机体 z 轴以有限角速度
     * 跟踪 s_cmd 方向（模拟低层姿态环带宽）。
     * 撞击倾角扰动绕过带宽限制注入，随后由带宽限制自然耗时改平——
     * 大翻倾时推力方向错误会持续一段时间（真实的高度/位置损失来源）。 */
    Vec3f s_cmd = vec3_add(ctrl->accel_cmd, vec3(0.0f, 0.0f, NAV_GRAVITY));
    Quatf q_des = quat_from_zdir_yaw(s_cmd, st->yaw);
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
    if (impact->active) {
        Quatf kick = quat_mul(quat_from_axis_angle(vec3(1.0f, 0.0f, 0.0f), impact->delta_roll),
                              quat_from_axis_angle(vec3(0.0f, 1.0f, 0.0f), impact->delta_pitch));
        q = quat_normalize(quat_mul(q, kick));
    }

    /* ---- 平移：推力沿真实机体 z 轴 ----
     * 翻倾期间推力方向错误 → 侧向/下坠加速度，这是翻倾恢复的真实代价 */
    float thrust = vec3_norm(s_cmd);
    Vec3f s_nav = vec3_scale(quat_rotate(q, vec3(0.0f, 0.0f, 1.0f)), thrust);
    Vec3f accel = vec3_sub(vec3_sub(s_nav, vec3(0.0f, 0.0f, NAV_GRAVITY)),
                           vec3_scale(st->vel, params->drag));
    st->vel = vec3_add(st->vel, vec3_scale(accel, dt));

    /* 撞击：接触冲量（瞬时速度改变） */
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

    /* 实际惯性加速度（含撞击冲量），供调试/日志 */
    st->last_accel = vec3_scale(vec3_sub(st->vel, vel_prev), 1.0f / dt);

    /* ---- IMU 真值 ----
     * 比力（机体系）= 推力通道 (0,0,thrust) + 撞击接触冲量 */
    {
        Quatf dq = quat_mul(quat_conj(st->att_prev), q);
        if (dq.w < 0.0f) { dq.x = -dq.x; dq.y = -dq.y; dq.z = -dq.z; }
        st->gyro_body = vec3_scale(vec3(dq.x, dq.y, dq.z), 2.0f / dt);
    }
    st->spec_force_body = vec3(0.0f, 0.0f, thrust);
    if (impact->active) {
        Vec3f pulse_nav = vec3_scale(impact->delta_v, 1.0f / dt);
        st->spec_force_body = vec3_add(st->spec_force_body,
                                       quat_rotate_inv(q, pulse_nav));
    }

    st->att = q;
    st->att_prev = q;
}
