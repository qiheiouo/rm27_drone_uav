#include <stdio.h>
#include "collision_interface.h"
#include "swarm_avoidance.h"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; \
} } while (0)

int main(void)
{
    TrajectoryMessage message;
    Vec3f position;
    Vec3f velocity;
    CollisionConfig collision;
    SwarmView swarm;
    Vec3f own_points[4];
    CollisionReport report;
    SwarmAvoidanceConfig avoidance_config;
    SwarmAvoidanceDecision avoidance;
    uint8_t index;

    trajectory_message_init(&message, 2u);
    message.valid = 1u;
    message.timestamp_ms = 1000u;
    message.validity_s = 0.5f;
    message.dt = 0.1f;
    message.point_count = 3u;
    for (index = 0u; index < message.point_count; index++) {
        message.points[index] = vec3((float)index, 0.0f, 1.0f);
        message.velocities[index] = vec3(10.0f, 0.0f, 0.0f);
    }
    CHECK(trajectory_message_is_fresh(&message, 1400u) == 1u);
    CHECK(trajectory_message_is_fresh(&message, 1600u) == 0u);
    CHECK(trajectory_message_evaluate(&message, 0.05f, &position, &velocity) == 1u);
    CHECK(fabsf(position.x - 0.5f) < 1e-4f);

    collision_init(&collision, 0.6f);
    swarm_view_init(&swarm, 2u);
    swarm.self.pos = vec3_zero();
    swarm.self.vel = vec3(1.0f, 0.0f, 0.0f);
    swarm.other_count = 1u;
    swarm.others[0].agent_id = 1u;
    swarm.others[0].valid = 1u;
    swarm.others[0].age_s = 0.0f;
    swarm.others[0].pos = vec3(0.4f, 0.0f, 0.0f);
    swarm.others[0].vel = vec3_zero();
    for (index = 0u; index < 4u; index++) {
        own_points[index] = vec3(0.1f * (float)index, 0.0f, 0.0f);
    }
    report = collision_check(&collision, own_points, 4u, &swarm);
    CHECK(report.conflict == 1u);
    CHECK(report.other_agent_id == 1u);

    swarm_avoidance_default_config(&avoidance_config);
    avoidance_config.mode = SWARM_ENABLED;
    avoidance = swarm_avoidance_decide(&avoidance_config, 2u, &report,
                                       vec3(1.0f, 0.0f, 0.0f));
    CHECK(avoidance.active == 1u);
    CHECK(avoidance.yielding == 1u);
    CHECK(vec3_norm(avoidance.velocity_bias) > 0.0f);
    avoidance = swarm_avoidance_decide(&avoidance_config, 0u, &report,
                                       vec3(1.0f, 0.0f, 0.0f));
    CHECK(avoidance.yielding == 0u);

    if (failures == 0) {
        printf("test_swarm: PASS\n");
        return 0;
    }
    printf("test_swarm: %d FAILURES\n", failures);
    return 1;
}
