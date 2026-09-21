#include <stdio.h>
#include <string.h>

#include "collision_interface.h"
#include "swarm_link.h"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; \
} } while (0)

static AgentState make_state(uint8_t id, uint16_t sequence, Vec3f position)
{
    AgentState state;
    agent_state_clear(&state);
    state.agent_id = id;
    state.sequence = sequence;
    state.timestamp_ms = 123456u + id;
    state.pos = position;
    state.vel = vec3(0.25f, -0.5f, 0.0f);
    state.quality = 0.8f;
    state.age_s = 0.0f;
    state.valid = 1u;
    return state;
}

int main(void)
{
    AgentState source = make_state(1u, 42u, vec3(1.0f, 2.0f, 3.0f));
    AgentState decoded;
    uint8_t frame[SWARM_LINK_STATE_FRAME_SIZE];
    uint8_t damaged[SWARM_LINK_STATE_FRAME_SIZE];
    SwarmPeerRegistry registry;
    SwarmPeerRegistry wrap_registry;
    SwarmView view;
    CollisionConfig collision;
    CollisionReport report;
    uint8_t id;

    CHECK(swarm_link_encode_state(&source, frame) == SWARM_LINK_OK);
    CHECK(frame[0] == (uint8_t)'R' && frame[1] == (uint8_t)'M');
    CHECK(frame[4] == SWARM_LINK_SCHEMA_VERSION);
    CHECK(frame[6] == SWARM_LINK_STATE_FRAME_SIZE && frame[7] == 0u);
    CHECK(swarm_link_decode_state(frame, sizeof(frame), &decoded) ==
          SWARM_LINK_OK);
    CHECK(decoded.agent_id == source.agent_id);
    CHECK(decoded.sequence == source.sequence);
    CHECK(decoded.timestamp_ms == source.timestamp_ms);
    CHECK(decoded.pos.x == source.pos.x && decoded.pos.y == source.pos.y &&
          decoded.pos.z == source.pos.z);
    CHECK(decoded.vel.x == source.vel.x && decoded.quality == source.quality);

    memcpy(damaged, frame, sizeof(damaged));
    damaged[20] ^= 0x40u;
    CHECK(swarm_link_decode_state(damaged, sizeof(damaged), &decoded) ==
          SWARM_LINK_ERROR_CRC);
    CHECK(swarm_link_decode_state(frame, sizeof(frame) - 1u, &decoded) ==
          SWARM_LINK_ERROR_SIZE);
    memcpy(damaged, frame, sizeof(damaged));
    damaged[4]++;
    CHECK(swarm_link_decode_state(damaged, sizeof(damaged), &decoded) ==
          SWARM_LINK_ERROR_VERSION);
    source.quality = 1.5f;
    CHECK(swarm_link_encode_state(&source, frame) == SWARM_LINK_ERROR_VALUE);
    source = make_state(1u, 10u, vec3(0.2f, 0.0f, 0.0f));

    swarm_peer_registry_init(&registry, 2u, 500u);
    CHECK(swarm_peer_registry_ingest(&registry, &source, 1000u) ==
          SWARM_PEER_ACCEPTED);
    CHECK(swarm_peer_registry_ingest(&registry, &source, 1010u) ==
          SWARM_PEER_REJECTED_DUPLICATE);
    source.sequence = 9u;
    CHECK(swarm_peer_registry_ingest(&registry, &source, 1020u) ==
          SWARM_PEER_REJECTED_OUT_OF_ORDER);
    source.sequence = 11u;
    CHECK(swarm_peer_registry_ingest(&registry, &source, 1030u) ==
          SWARM_PEER_UPDATED);
    source = make_state(2u, 1u, vec3_zero());
    CHECK(swarm_peer_registry_ingest(&registry, &source, 1030u) ==
          SWARM_PEER_REJECTED_SELF);

    for (id = 3u; id <= 5u; id++) {
        source = make_state(id, 1u, vec3((float)id, 0.0f, 0.0f));
        CHECK(swarm_peer_registry_ingest(&registry, &source, 1040u) ==
              SWARM_PEER_ACCEPTED);
    }
    source = make_state(6u, 1u, vec3(6.0f, 0.0f, 0.0f));
    CHECK(swarm_peer_registry_ingest(&registry, &source, 1050u) ==
          SWARM_PEER_REJECTED_CAPACITY);
    swarm_peer_registry_build_view(&registry, 0, 1230u, &view);
    CHECK(view.other_count == SWARM_MAX_OTHER_AGENTS);
    CHECK(view.others[0].age_s > 0.19f && view.others[0].age_s < 0.21f);

    collision_init(&collision, 0.6f);
    view.self.pos = vec3_zero();
    view.self.vel = vec3_zero();
    report = collision_check(&collision, 0, 0u, &view);
    CHECK(report.conflict == 1u);
    CHECK(report.other_agent_id == 1u);

    swarm_peer_registry_build_view(&registry, 0, 1600u, &view);
    CHECK(view.other_count == 0u);
    CHECK(registry.stats.expired_peers == SWARM_MAX_OTHER_AGENTS);
    CHECK(swarm_peer_registry_ingest(&registry, &source, 1600u) ==
          SWARM_PEER_ACCEPTED);

    swarm_peer_registry_init(&wrap_registry, 2u, 500u);
    source = make_state(1u, 65535u, vec3_zero());
    CHECK(swarm_peer_registry_ingest(&wrap_registry, &source, 0xfffffff0u) ==
          SWARM_PEER_ACCEPTED);
    source.sequence = 0u;
    CHECK(swarm_peer_registry_ingest(&wrap_registry, &source, 0x00000010u) ==
          SWARM_PEER_UPDATED);
    swarm_peer_registry_build_view(&wrap_registry, 0, 0x00000020u, &view);
    CHECK(view.other_count == 1u);
    CHECK(view.others[0].sequence == 0u);
    CHECK(view.others[0].age_s > 0.015f && view.others[0].age_s < 0.017f);

    CHECK(registry.stats.accepted_frames == 6u);
    CHECK(registry.stats.duplicate_frames == 1u);
    CHECK(registry.stats.out_of_order_frames == 1u);
    CHECK(registry.stats.self_frames == 1u);
    CHECK(registry.stats.capacity_drops == 1u);

    if (failures == 0) {
        printf("test_swarm_link: PASS\n");
        return 0;
    }
    printf("test_swarm_link: %d FAILURES\n", failures);
    return 1;
}
