/* Versioned wire protocol and fixed-capacity peer registry for swarm state. */
#ifndef SWARM_LINK_H
#define SWARM_LINK_H

#include <stdint.h>

#include "agent_state.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SWARM_LINK_SCHEMA_VERSION 1u
#define SWARM_LINK_FRAME_TYPE_STATE 1u
#define SWARM_LINK_STATE_FRAME_SIZE 48u
#define SWARM_LINK_FLAG_VALID 1u

typedef enum {
    SWARM_LINK_OK = 0,
    SWARM_LINK_ERROR_ARGUMENT,
    SWARM_LINK_ERROR_SIZE,
    SWARM_LINK_ERROR_MAGIC,
    SWARM_LINK_ERROR_VERSION,
    SWARM_LINK_ERROR_TYPE,
    SWARM_LINK_ERROR_FLAGS,
    SWARM_LINK_ERROR_CRC,
    SWARM_LINK_ERROR_VALUE
} SwarmLinkStatus;

typedef enum {
    SWARM_PEER_ACCEPTED = 0,
    SWARM_PEER_UPDATED,
    SWARM_PEER_REJECTED_INVALID,
    SWARM_PEER_REJECTED_SELF,
    SWARM_PEER_REJECTED_DUPLICATE,
    SWARM_PEER_REJECTED_OUT_OF_ORDER,
    SWARM_PEER_REJECTED_CAPACITY
} SwarmPeerResult;

typedef struct {
    uint32_t accepted_frames;
    uint32_t duplicate_frames;
    uint32_t out_of_order_frames;
    uint32_t self_frames;
    uint32_t invalid_frames;
    uint32_t capacity_drops;
    uint32_t expired_peers;
} SwarmPeerStats;

typedef struct {
    AgentState peers[SWARM_MAX_OTHER_AGENTS];
    uint32_t received_at_ms[SWARM_MAX_OTHER_AGENTS];
    uint8_t occupied[SWARM_MAX_OTHER_AGENTS];
    uint8_t self_agent_id;
    uint32_t max_age_ms;
    SwarmPeerStats stats;
} SwarmPeerRegistry;

uint32_t swarm_link_crc32(const uint8_t *data, uint32_t size);
SwarmLinkStatus swarm_link_encode_state(
    const AgentState *state,
    uint8_t frame[SWARM_LINK_STATE_FRAME_SIZE]);
SwarmLinkStatus swarm_link_decode_state(const uint8_t *frame,
                                        uint16_t size,
                                        AgentState *state);

void swarm_peer_registry_init(SwarmPeerRegistry *registry,
                              uint8_t self_agent_id,
                              uint32_t max_age_ms);
SwarmPeerResult swarm_peer_registry_ingest(SwarmPeerRegistry *registry,
                                           const AgentState *state,
                                           uint32_t received_at_ms);
void swarm_peer_registry_build_view(SwarmPeerRegistry *registry,
                                    const AgentState *self,
                                    uint32_t now_ms,
                                    SwarmView *view);

#ifdef __cplusplus
}
#endif

#endif /* SWARM_LINK_H */
