#include "trajectory_message.h"

void trajectory_message_init(TrajectoryMessage *message, uint8_t agent_id)
{
    uint8_t index;
    message->agent_id = agent_id;
    message->sequence = 0u;
    message->timestamp_ms = 0u;
    message->point_count = 0u;
    message->dt = 0.1f;
    message->duration_s = 0.0f;
    message->validity_s = 0.5f;
    message->valid = 0u;
    for (index = 0u; index < SWARM_TRAJ_MAX_POINTS; index++) {
        message->points[index] = vec3_zero();
        message->velocities[index] = vec3_zero();
    }
}

uint8_t trajectory_message_is_fresh(const TrajectoryMessage *message,
                                    uint32_t now_ms)
{
    float age_s;
    if (!message->valid || message->point_count == 0u ||
        message->point_count > SWARM_TRAJ_MAX_POINTS || message->dt <= 0.0f) {
        return 0u;
    }
    age_s = now_ms >= message->timestamp_ms
        ? (float)(now_ms - message->timestamp_ms) * 0.001f : 0.0f;
    return age_s <= message->validity_s ? 1u : 0u;
}

uint8_t trajectory_message_evaluate(const TrajectoryMessage *message,
                                    float future_time_s,
                                    Vec3f *position,
                                    Vec3f *velocity)
{
    float index_float;
    uint8_t index;
    float fraction;
    if (!message->valid || message->point_count == 0u || message->dt <= 0.0f) {
        *position = vec3_zero();
        *velocity = vec3_zero();
        return 0u;
    }
    if (message->point_count == 1u || future_time_s <= 0.0f) {
        *position = message->points[0];
        *velocity = message->velocities[0];
        return 1u;
    }
    index_float = future_time_s / message->dt;
    index = (uint8_t)index_float;
    if (index >= message->point_count - 1u) {
        *position = message->points[message->point_count - 1u];
        *velocity = message->velocities[message->point_count - 1u];
        return 1u;
    }
    fraction = index_float - (float)index;
    *position = vec3_lerp(message->points[index], message->points[index + 1u], fraction);
    *velocity = vec3_lerp(message->velocities[index], message->velocities[index + 1u], fraction);
    return 1u;
}
