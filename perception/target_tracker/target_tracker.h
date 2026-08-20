/* Fixed-memory alpha-beta target tracker for target-relative guidance. */
#ifndef TARGET_TRACKER_H
#define TARGET_TRACKER_H

#include <stdint.h>
#include "nav_math.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TARGET_TRACK_LOST = 0,
    TARGET_TRACK_TRACKING,
    TARGET_TRACK_COASTING
} TargetTrackStatus;

typedef struct {
    uint8_t  visible;
    Vec3f    rel_pos;          /* Target relative to vehicle in navigation frame (m). */
    Vec3f    bearing;          /* Optional; derived from rel_pos when omitted. */
    float    confidence;       /* [0,1]; <=0 is treated as legacy confidence 1. */
    uint32_t timestamp_ms;
} TargetObs;

typedef struct {
    uint8_t  visible;
    TargetTrackStatus status;
    Vec3f    rel_pos;
    Vec3f    rel_vel;
    Vec3f    bearing;
    float    range;
    float    confidence;
    float    time_since_update;
    float    age;
    uint32_t timestamp_ms;
} TargetTrack;

typedef struct {
    float alpha;
    float beta;
    float max_innovation_m;
    float coast_timeout_s;
    float lost_timeout_s;
    float confidence_decay_per_s;
} TargetTrackerConfig;

typedef struct {
    TargetTrackerConfig cfg;
    TargetTrack out;
    float lost_time;
    uint8_t initialized;
} TargetTracker;

void target_tracker_default_config(TargetTrackerConfig *cfg);
void target_tracker_init_config(TargetTracker *trk, const TargetTrackerConfig *cfg);
void target_tracker_init(TargetTracker *trk, float filter_cutoff_hz, float dt);
void target_tracker_update(TargetTracker *trk, const TargetObs *obs, float dt);
void target_tracker_reset(TargetTracker *trk);
const char *target_track_status_name(TargetTrackStatus status);

#ifdef __cplusplus
}
#endif

#endif /* TARGET_TRACKER_H */
