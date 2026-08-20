/* Fixed-memory guidance laws and hierarchical terminal guidance. */
#ifndef GUIDANCE_H
#define GUIDANCE_H

#include <stdint.h>
#include "nav_math.h"
#include "state_estimator.h"
#include "target_tracker.h"
#include "home_detector.h"
#include "waypoint.h"
#include "trajectory.h"
#include "pos_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TERMINAL_PURSUIT = 0,
    TERMINAL_CLOSING,
    TERMINAL_FINAL_ALIGN,
    TERMINAL_CONTACT_APPROACH,
    TERMINAL_REACQUIRE,
    TERMINAL_FAILED
} TerminalStage;

typedef struct {
    float approach_speed;       /* Compatibility: overall speed cap. */
    float kp;                   /* Compatibility: horizontal position gain. */
    float min_closing_speed;
    float closing_range_m;
    float final_align_range_m;
    float contact_range_m;
    float closing_speed_mps;
    float final_speed_mps;
    float contact_speed_mps;
    float kp_vertical;
    float max_vertical_speed_mps;
    float max_accel_mps2;
    float yaw_align_tolerance_rad;
    float min_confidence;
    float loss_grace_s;
    float reacquire_timeout_s;
} TerminalParams;

typedef struct {
    TerminalParams cfg;
    TerminalStage stage;
    Vec3f previous_velocity;
    float stage_time;
    float lost_time;
    uint8_t contact_expected;
    uint8_t failed;
} TerminalGuidance;

typedef struct {
    float search_alt;
    float descend_speed;
    float kp_lateral;
    float max_lateral_speed;
    float lateral_tol;
    float spiral_rate;
    float spiral_max_radius;
    float spiral_omega;
    float blind_land_alt;
    float blind_land_timeout;
    float kp_yaw;
    float yaw_tolerance_rad;
} HomeParams;

void terminal_guidance_init(TerminalGuidance *guidance, const TerminalParams *params);
void terminal_guidance_reset(TerminalGuidance *guidance);
void terminal_guidance_update(TerminalGuidance *guidance,
                              const TargetTrack *target,
                              const NavState *nav,
                              float dt,
                              GuidanceOutput *out);
const char *terminal_stage_name(TerminalStage stage);

void guidance_takeoff(const Vec3f *target_pos, const NavState *nav, GuidanceOutput *out);
void guidance_hold(const Vec3f *hold_pos, const NavState *nav, GuidanceOutput *out);
void cruise_guidance_update(const Waypoint *wp, const NavState *nav, GuidanceOutput *out);
void trajectory_guidance_update(const Trajectory *traj, float t,
                                const NavState *nav, GuidanceOutput *out);
/* Stateless compatibility wrapper. New integrations should use TerminalGuidance. */
void target_guidance_update(const TargetTrack *target, const NavState *nav,
                            const TerminalParams *params, GuidanceOutput *out);
void recovery_guidance_update(const NavState *nav, float damping_gain, GuidanceOutput *out);
void home_guidance_update(const HomeTrack *home, const NavState *nav,
                          const Vec3f *home_est, const HomeParams *params,
                          GuidanceOutput *out);
void guidance_land(const NavState *nav, float descend_speed, GuidanceOutput *out);

#ifdef __cplusplus
}
#endif

#endif /* GUIDANCE_H */
