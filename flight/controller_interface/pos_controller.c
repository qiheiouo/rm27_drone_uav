#include "pos_controller.h"

void pos_controller_update(const PosCtrlParams *params,
                           const GuidanceOutput *sp,
                           const NavState *nav,
                           CtrlOutput *out)
{
    Vec3f vel_cmd = sp->vel_sp;

    if (sp->use_pos_sp) {
        Vec3f pos_err = vec3_sub(sp->pos_sp, nav->pos);
        Vec3f vel_from_pos = vec3_clamp_norm(vec3_scale(pos_err, params->kp_pos),
                                             params->max_vel);
        vel_cmd = vec3_add(vel_cmd, vel_from_pos);
    }
    vel_cmd = vec3_clamp_norm(vel_cmd, params->max_vel);

    Vec3f vel_err = vec3_sub(vel_cmd, nav->vel);
    out->accel_cmd = vec3_clamp_norm(vec3_scale(vel_err, params->kp_vel),
                                     params->max_accel);

    float yaw_err = wrap_pi(sp->yaw_sp - nav->yaw);
    out->yaw_rate_cmd = clampf(params->kp_yaw * yaw_err,
                               -params->max_yaw_rate, params->max_yaw_rate);
}
