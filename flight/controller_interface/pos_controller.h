/* Navigation-to-flight-control interface. */
#ifndef POS_CONTROLLER_H
#define POS_CONTROLLER_H

#include <stdint.h>
#include "nav_math.h"
#include "state_estimator.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    Vec3f   pos_sp;
    Vec3f   vel_sp;
    Vec3f   accel_sp;
    float   yaw_sp;
    uint8_t use_pos_sp;
    uint8_t use_accel_sp;
} GuidanceOutput;

typedef struct {
    Vec3f accel_cmd;
    float yaw_rate_cmd;
} CtrlOutput;

typedef struct {
    float kp_pos;
    float kp_vel;
    float max_vel;
    float max_accel;
    float kp_yaw;
    float max_yaw_rate;
} PosCtrlParams;

void guidance_output_hold(GuidanceOutput *out, const NavState *nav);
void pos_controller_update(const PosCtrlParams *params,
                           const GuidanceOutput *sp,
                           const NavState *nav,
                           CtrlOutput *out);

#ifdef __cplusplus
}
#endif

#endif /* POS_CONTROLLER_H */
