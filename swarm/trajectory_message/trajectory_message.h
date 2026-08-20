/* Compact fixed-length future trajectory message. */
#ifndef TRAJECTORY_MESSAGE_H
#define TRAJECTORY_MESSAGE_H

#include <stdint.h>
#include "nav_math.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SWARM_TRAJ_MAX_POINTS 8u

typedef struct {
    uint8_t  agent_id;
    uint16_t sequence;
    uint32_t timestamp_ms;
    uint8_t  point_count;
    float    dt;
    float    duration_s;
    Vec3f    points[SWARM_TRAJ_MAX_POINTS];
    Vec3f    velocities[SWARM_TRAJ_MAX_POINTS];
    float    validity_s;
    uint8_t  valid;
} TrajectoryMessage;

void trajectory_message_init(TrajectoryMessage *message, uint8_t agent_id);
uint8_t trajectory_message_is_fresh(const TrajectoryMessage *message,
                                    uint32_t now_ms);
uint8_t trajectory_message_evaluate(const TrajectoryMessage *message,
                                    float future_time_s,
                                    Vec3f *position,
                                    Vec3f *velocity);

#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_MESSAGE_H */
