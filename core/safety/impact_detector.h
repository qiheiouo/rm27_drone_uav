/* Multi-channel, armed and debounced impact detector. */
#ifndef IMPACT_DETECTOR_H
#define IMPACT_DETECTOR_H

#include <stdint.h>
#include "nav_math.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    Vec3f accel;
    Vec3f gyro;
    uint32_t timestamp_ms;
} ImuSample;

typedef enum {
    NO_IMPACT = 0,
    POSSIBLE_IMPACT,
    CONFIRMED_IMPACT
} ImpactState;

typedef struct {
    float accel_spike_threshold;
    float gyro_spike_threshold;
    uint8_t confirm_samples;
} ImpactDetectorConfig;

typedef struct {
    float velocity_jump_threshold_mps;
    float attitude_jump_threshold_rad;
    float debounce_s;
    float refractory_s;
    float minimum_confidence;
} ImpactFusionConfig;

typedef struct {
    ImpactState state;
    float confidence;
    uint8_t accel_evidence;
    uint8_t gyro_evidence;
    uint8_t velocity_evidence;
    uint8_t attitude_evidence;
    uint8_t rising_edge;
} ImpactReport;

typedef struct {
    ImpactDetectorConfig cfg;
    ImpactFusionConfig fusion;
    uint8_t triggered;
    uint8_t confirm_count;
    uint8_t armed;
    uint8_t have_previous_motion;
    float candidate_time;
    float refractory_remaining;
    Vec3f previous_velocity;
    Quatf previous_attitude;
    uint32_t previous_timestamp_ms;
    ImpactReport report;
} ImpactDetector;

void impact_detector_init(ImpactDetector *det, const ImpactDetectorConfig *cfg);
void impact_detector_default_fusion_config(ImpactFusionConfig *cfg);
void impact_detector_configure_fusion(ImpactDetector *det,
                                      const ImpactFusionConfig *cfg);
void impact_detector_arm(ImpactDetector *det, uint8_t armed);
void impact_detector_reset(ImpactDetector *det);
uint8_t impact_detector_update(ImpactDetector *det, const ImuSample *sample);
ImpactReport impact_detector_update_fused(ImpactDetector *det,
                                          const ImuSample *sample,
                                          Vec3f velocity,
                                          Quatf attitude,
                                          float dt);
const char *impact_state_name(ImpactState state);

#ifdef __cplusplus
}
#endif

#endif /* IMPACT_DETECTOR_H */
