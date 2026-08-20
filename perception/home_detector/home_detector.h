/* Fixed-memory home-marker observation filter. */
#ifndef HOME_DETECTOR_H
#define HOME_DETECTOR_H

#include <stdint.h>
#include "nav_math.h"
#include "lowpass.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  visible;
    float    confidence;
    Vec3f    rel_pos;
    float    relative_yaw;
    uint32_t timestamp_ms;
} HomeObs;

typedef struct {
    uint8_t  visible;
    float    confidence;
    Vec3f    rel_pos;
    float    relative_yaw;
    float    time_since_update;
    uint32_t timestamp_ms;
} HomeTrack;

typedef struct {
    LowPass1 fx, fy, fz, fyaw;
    float lost_time;
    float confidence_decay_per_s;
    HomeTrack out;
} HomeDetector;

void home_detector_init(HomeDetector *det, float filter_cutoff_hz, float dt);
void home_detector_update(HomeDetector *det, const HomeObs *obs, float dt);
void home_detector_reset(HomeDetector *det);

#ifdef __cplusplus
}
#endif

#endif /* HOME_DETECTOR_H */
