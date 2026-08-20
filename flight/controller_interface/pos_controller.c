#include "pos_controller.h"

void guidance_output_hold(GuidanceOutput *out, const NavState *nav)
{
    out->pos_sp = nav->pos;
    out->vel_sp = vec3_zero();
    out->accel_sp = vec3_zero();
    out->yaw_sp = nav->yaw;
    out->use_pos_sp = 1u;
    out->use_accel_sp = 0u;
}

void pos_controller_update(const PosCtrlParams *params,
                           const GuidanceOutput *sp,
                           const NavState *nav,
                           CtrlOutput *out)
{
    Vec3f vel_cmd = sp->vel_sp;
    Vec3f vel_error;

    if (sp->use_pos_sp) {
        Vec3f pos_error = vec3_sub(sp->pos_sp, nav->pos);
        Vec3f vel_from_pos = vec3_clamp_norm(vec3_scale(pos_error, params->kp_pos),
                                             params->max_vel);
        vel_cmd = vec3_add(vel_cmd, vel_from_pos);
    }
    vel_cmd = vec3_clamp_norm(vel_cmd, params->max_vel);

    vel_error = vec3_sub(vel_cmd, nav->vel);
    out->accel_cmd = vec3_scale(vel_error, params->kp_vel);
    if (sp->use_accel_sp) {
        out->accel_cmd = vec3_add(out->accel_cmd, sp->accel_sp);
    }
    out->accel_cmd = vec3_clamp_norm(out->accel_cmd, params->max_accel);
    out->yaw_rate_cmd = clampf(params->kp_yaw * wrap_pi(sp->yaw_sp - nav->yaw),
                               -params->max_yaw_rate, params->max_yaw_rate);
}
