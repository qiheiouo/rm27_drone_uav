#include <string.h>

#include "swarm_link.h"

#define SWARM_LINK_CRC_OFFSET 44u

typedef char swarm_link_requires_32_bit_float[
    (sizeof(float) == 4u) ? 1 : -1];

static void put_u16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xffu);
    dst[1] = (uint8_t)((value >> 8) & 0xffu);
}

static void put_u32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xffu);
    dst[1] = (uint8_t)((value >> 8) & 0xffu);
    dst[2] = (uint8_t)((value >> 16) & 0xffu);
    dst[3] = (uint8_t)((value >> 24) & 0xffu);
}

static uint16_t get_u16(const uint8_t *src)
{
    return (uint16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8));
}

static uint32_t get_u32(const uint8_t *src)
{
    return (uint32_t)src[0] |
           ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) |
           ((uint32_t)src[3] << 24);
}

static void put_float(uint8_t *dst, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    put_u32(dst, bits);
}

static float get_float(const uint8_t *src)
{
    uint32_t bits = get_u32(src);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static uint8_t agent_state_valid(const AgentState *state)
{
    return (state != 0 && state->valid &&
            vec3_is_finite(state->pos) && vec3_is_finite(state->vel) &&
            nav_isfinite(state->quality) &&
            state->quality >= 0.0f && state->quality <= 1.0f) ? 1u : 0u;
}

static uint8_t sequence_newer(uint16_t candidate, uint16_t reference)
{
    uint16_t delta = (uint16_t)(candidate - reference);
    return (delta != 0u && delta < 0x8000u) ? 1u : 0u;
}

uint32_t swarm_link_crc32(const uint8_t *data, uint32_t size)
{
    uint32_t crc = 0xffffffffu;
    uint32_t index;
    if (data == 0 && size != 0u) return 0u;
    for (index = 0u; index < size; index++) {
        uint8_t bit;
        crc ^= data[index];
        for (bit = 0u; bit < 8u; bit++) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xedb88320u & mask);
        }
    }
    return crc ^ 0xffffffffu;
}

SwarmLinkStatus swarm_link_encode_state(
    const AgentState *state,
    uint8_t frame[SWARM_LINK_STATE_FRAME_SIZE])
{
    if (state == 0 || frame == 0) return SWARM_LINK_ERROR_ARGUMENT;
    if (!agent_state_valid(state)) return SWARM_LINK_ERROR_VALUE;

    memset(frame, 0, SWARM_LINK_STATE_FRAME_SIZE);
    frame[0] = (uint8_t)'R';
    frame[1] = (uint8_t)'M';
    frame[2] = (uint8_t)'S';
    frame[3] = (uint8_t)'W';
    frame[4] = SWARM_LINK_SCHEMA_VERSION;
    frame[5] = SWARM_LINK_FRAME_TYPE_STATE;
    put_u16(&frame[6], SWARM_LINK_STATE_FRAME_SIZE);
    frame[8] = state->agent_id;
    frame[9] = SWARM_LINK_FLAG_VALID;
    put_u16(&frame[10], state->sequence);
    put_u32(&frame[12], state->timestamp_ms);
    put_float(&frame[16], state->pos.x);
    put_float(&frame[20], state->pos.y);
    put_float(&frame[24], state->pos.z);
    put_float(&frame[28], state->vel.x);
    put_float(&frame[32], state->vel.y);
    put_float(&frame[36], state->vel.z);
    put_float(&frame[40], state->quality);
    put_u32(&frame[SWARM_LINK_CRC_OFFSET],
            swarm_link_crc32(frame, SWARM_LINK_CRC_OFFSET));
    return SWARM_LINK_OK;
}

SwarmLinkStatus swarm_link_decode_state(const uint8_t *frame,
                                        uint16_t size,
                                        AgentState *state)
{
    uint32_t expected_crc;
    if (frame == 0 || state == 0) return SWARM_LINK_ERROR_ARGUMENT;
    agent_state_clear(state);
    if (size != SWARM_LINK_STATE_FRAME_SIZE ||
        get_u16(&frame[6]) != SWARM_LINK_STATE_FRAME_SIZE) {
        return SWARM_LINK_ERROR_SIZE;
    }
    if (frame[0] != (uint8_t)'R' || frame[1] != (uint8_t)'M' ||
        frame[2] != (uint8_t)'S' || frame[3] != (uint8_t)'W') {
        return SWARM_LINK_ERROR_MAGIC;
    }
    if (frame[4] != SWARM_LINK_SCHEMA_VERSION) {
        return SWARM_LINK_ERROR_VERSION;
    }
    if (frame[5] != SWARM_LINK_FRAME_TYPE_STATE) {
        return SWARM_LINK_ERROR_TYPE;
    }
    if (frame[9] != SWARM_LINK_FLAG_VALID) {
        return SWARM_LINK_ERROR_FLAGS;
    }
    expected_crc = swarm_link_crc32(frame, SWARM_LINK_CRC_OFFSET);
    if (get_u32(&frame[SWARM_LINK_CRC_OFFSET]) != expected_crc) {
        return SWARM_LINK_ERROR_CRC;
    }

    state->agent_id = frame[8];
    state->sequence = get_u16(&frame[10]);
    state->timestamp_ms = get_u32(&frame[12]);
    state->pos = vec3(get_float(&frame[16]), get_float(&frame[20]),
                      get_float(&frame[24]));
    state->vel = vec3(get_float(&frame[28]), get_float(&frame[32]),
                      get_float(&frame[36]));
    state->quality = get_float(&frame[40]);
    state->age_s = 0.0f;
    state->valid = 1u;
    if (!agent_state_valid(state)) {
        agent_state_clear(state);
        return SWARM_LINK_ERROR_VALUE;
    }
    return SWARM_LINK_OK;
}

void swarm_peer_registry_init(SwarmPeerRegistry *registry,
                              uint8_t self_agent_id,
                              uint32_t max_age_ms)
{
    uint8_t index;
    memset(registry, 0, sizeof(*registry));
    registry->self_agent_id = self_agent_id;
    registry->max_age_ms = max_age_ms > 0u ? max_age_ms : 1u;
    for (index = 0u; index < SWARM_MAX_OTHER_AGENTS; index++) {
        agent_state_clear(&registry->peers[index]);
    }
}

static void expire_peers(SwarmPeerRegistry *registry, uint32_t now_ms)
{
    uint8_t index;
    for (index = 0u; index < SWARM_MAX_OTHER_AGENTS; index++) {
        if (registry->occupied[index] &&
            (uint32_t)(now_ms - registry->received_at_ms[index]) >
                registry->max_age_ms) {
            registry->occupied[index] = 0u;
            agent_state_clear(&registry->peers[index]);
            registry->stats.expired_peers++;
        }
    }
}

SwarmPeerResult swarm_peer_registry_ingest(SwarmPeerRegistry *registry,
                                           const AgentState *state,
                                           uint32_t received_at_ms)
{
    uint8_t index;
    uint8_t free_index = SWARM_MAX_OTHER_AGENTS;
    if (registry == 0 || !agent_state_valid(state)) {
        if (registry != 0) registry->stats.invalid_frames++;
        return SWARM_PEER_REJECTED_INVALID;
    }
    if (state->agent_id == registry->self_agent_id) {
        registry->stats.self_frames++;
        return SWARM_PEER_REJECTED_SELF;
    }

    expire_peers(registry, received_at_ms);
    for (index = 0u; index < SWARM_MAX_OTHER_AGENTS; index++) {
        if (!registry->occupied[index]) {
            if (free_index == SWARM_MAX_OTHER_AGENTS) free_index = index;
            continue;
        }
        if (registry->peers[index].agent_id == state->agent_id) {
            if (state->sequence == registry->peers[index].sequence) {
                registry->stats.duplicate_frames++;
                return SWARM_PEER_REJECTED_DUPLICATE;
            }
            if (!sequence_newer(state->sequence,
                                registry->peers[index].sequence)) {
                registry->stats.out_of_order_frames++;
                return SWARM_PEER_REJECTED_OUT_OF_ORDER;
            }
            registry->peers[index] = *state;
            registry->peers[index].age_s = 0.0f;
            registry->received_at_ms[index] = received_at_ms;
            registry->stats.accepted_frames++;
            return SWARM_PEER_UPDATED;
        }
    }

    if (free_index == SWARM_MAX_OTHER_AGENTS) {
        registry->stats.capacity_drops++;
        return SWARM_PEER_REJECTED_CAPACITY;
    }
    registry->peers[free_index] = *state;
    registry->peers[free_index].age_s = 0.0f;
    registry->received_at_ms[free_index] = received_at_ms;
    registry->occupied[free_index] = 1u;
    registry->stats.accepted_frames++;
    return SWARM_PEER_ACCEPTED;
}

void swarm_peer_registry_build_view(SwarmPeerRegistry *registry,
                                    const AgentState *self,
                                    uint32_t now_ms,
                                    SwarmView *view)
{
    uint8_t index;
    swarm_view_init(view, registry->self_agent_id);
    if (self != 0 && agent_state_valid(self) &&
        self->agent_id == registry->self_agent_id) {
        view->self = *self;
    }
    expire_peers(registry, now_ms);
    for (index = 0u; index < SWARM_MAX_OTHER_AGENTS; index++) {
        if (registry->occupied[index]) {
            AgentState state = registry->peers[index];
            state.age_s = (float)(uint32_t)(
                now_ms - registry->received_at_ms[index]) * 0.001f;
            view->others[view->other_count++] = state;
        }
    }
}
