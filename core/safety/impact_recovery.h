/* Hierarchical post-impact recovery controller. */
#ifndef IMPACT_RECOVERY_H
#define IMPACT_RECOVERY_H

#include <stdint.h>
#include "nav_math.h"
#include "state_estimator.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RECOVERY_IDLE = 0,
    RECOVERY_VISUAL_HOLD,
    RECOVERY_ATTITUDE,
    RECOVERY_VELOCITY_DAMPING,
    RECOVERY_ESTIMATOR_CHECK,
    RECOVERY_BREAKAWAY,
    RECOVERY_COMPLETE,
    RECOVERY_FAILED
} RecoveryStage;

typedef struct {
    float visual_hold_s;
    float attitude_timeout_s;
    float velocity_timeout_s;
    float estimator_timeout_s;
    float total_timeout_s;
    float tilt_ok_rad;
    float speed_ok_mps;
    float damping_gain;
    float climb_speed_mps;
    float breakaway_height_m;
    float breakaway_tolerance_m;
} ImpactRecoveryConfig;

typedef struct {
    RecoveryStage stage;
    Vec3f hold_position;
    Vec3f desired_velocity;
    Vec3f breakaway_target;
    uint8_t use_position;
    uint8_t ready_for_breakaway;
    uint8_t complete;
    uint8_t failed;
    uint8_t emergency_fallback;
} ImpactRecoveryOutput;

typedef struct {
    ImpactRecoveryConfig cfg;
    RecoveryStage stage;
    float stage_time;
    float total_time;
    Vec3f hold_position;
    Vec3f breakaway_target;
    uint8_t active;
} ImpactRecovery;

void impact_recovery_default_config(ImpactRecoveryConfig *cfg);
void impact_recovery_init(ImpactRecovery *recovery,
                          const ImpactRecoveryConfig *cfg);
void impact_recovery_start(ImpactRecovery *recovery, const NavState *nav);
void impact_recovery_update(ImpactRecovery *recovery,
                            const NavState *nav,
                            uint8_t visual_available,
                            float dt,
                            ImpactRecoveryOutput *out);
const char *recovery_stage_name(RecoveryStage stage);

#ifdef __cplusplus
}
#endif

#endif /* IMPACT_RECOVERY_H */
