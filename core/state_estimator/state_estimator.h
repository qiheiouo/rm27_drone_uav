/* Lightweight visual-inertial state framework with explicit health states. */
#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <stdint.h>
#include "nav_math.h"
#include "impact_detector.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EST_INIT = 0,
    EST_IMU_ONLY,
    EST_VISUAL_AIDED,
    EST_TRACKING,
    EST_DEGRADED,
    EST_LOST,
    EST_RECOVERING,
    EST_RELOCALIZED
} EstStatus;

typedef enum {
    EST_MODE_TRUTH = 0,
    EST_MODE_INS
} EstimatorMode;

typedef enum {
    NAV_VALID_INVALID = 0,
    NAV_VALID_DEGRADED,
    NAV_VALID_VALID
} NavValidity;

typedef struct {
    Vec3f    pos;
    Vec3f    vel;
    Quatf    att;
    Vec3f    angular_velocity;
    Vec3f    linear_acceleration;
    float    yaw;
    float    yaw_rate;
    uint32_t timestamp_ms;
    NavValidity validity;
    float    quality;
    EstimatorMode mode;
    EstStatus status;
} NavState;

typedef struct {
    Vec3f    pos;
    Vec3f    vel;
    float    yaw;
    float    yaw_rate;
    Quatf    att;
    uint8_t  valid;
    uint32_t timestamp_ms;
} OdomSample;

typedef struct {
    uint8_t valid;
    Vec3f position;
    Vec3f velocity;
    float position_gain;
    float velocity_gain;
    float max_position_innovation_m;
    float max_velocity_innovation_mps;
    uint32_t timestamp_ms;
} VisualCorrection;

typedef struct {
    EstimatorMode mode;
    float  vo_degraded_after_s;
    float  vo_lost_after_s;
    float  recovering_hold_s;
    float  lost_timeout_s;
    float  impact_blind_s;
    uint8_t lost_on_impact;
    float  kp_tilt;
    float  ki_gyro_bias;
    float  kp_vo_pos;
    float  kp_vo_vel;
    float  kp_vo_yaw;
    float  kp_tof;
    float  kp_tof_vel;
} EstimatorConfig;

typedef struct {
    EstimatorConfig cfg;
    NavState  out;
    float     timer;
    float     vo_invalid_time;
    float     blind_time;
    NavState  last_valid;
    Vec3f     ins_pos;
    Vec3f     ins_vel;
    Quatf     ins_att;
    Vec3f     gyro_bias;
    Vec3f     accel_bias;
    uint16_t  good_visual_frames;
    uint16_t  rejected_measurements;
    float     max_position_innovation_m;
    float     max_velocity_innovation_mps;
} StateEstimator;

void estimator_init(StateEstimator *est, const EstimatorConfig *cfg);
void estimator_notify_impact(StateEstimator *est);
void estimator_notify_relocalized(StateEstimator *est, const NavState *absolute_ref);
uint8_t estimator_apply_visual_correction(StateEstimator *est,
                                          const VisualCorrection *correction,
                                          float dt);
void estimator_update(StateEstimator *est, const ImuSample *imu,
                      const OdomSample *vo, float tof_height, float dt);
const char *est_status_name(EstStatus status);

#ifdef __cplusplus
}
#endif

#endif /* STATE_ESTIMATOR_H */
