/* Deterministic fixed-memory broadcast network used by host swarm tests. */
#ifndef SIM_SWARM_NETWORK_H
#define SIM_SWARM_NETWORK_H

#include <stdint.h>

#include "swarm_link.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SIM_SWARM_NETWORK_MAX_NODES 4u
#define SIM_SWARM_NETWORK_QUEUE_CAPACITY 64u
#define SIM_SWARM_NETWORK_MAX_FRAME_SIZE SWARM_LINK_STATE_FRAME_SIZE

typedef enum {
    SIM_SWARM_NETWORK_OK = 0,
    SIM_SWARM_NETWORK_ERROR_ARGUMENT,
    SIM_SWARM_NETWORK_ERROR_NODE,
    SIM_SWARM_NETWORK_ERROR_CAPACITY,
    SIM_SWARM_NETWORK_ERROR_FRAME
} SimSwarmNetworkStatus;

typedef struct {
    uint32_t base_delay_ms;
    uint32_t jitter_ms;
    uint32_t reorder_extra_delay_ms;
    uint32_t outage_start_ms;
    uint32_t outage_end_ms;
    uint32_t seed;
    uint16_t drop_every_n;
    uint16_t duplicate_every_n;
    uint16_t reorder_every_n;
} SimSwarmNetworkConfig;

typedef struct {
    uint32_t broadcasts;
    uint32_t delivery_attempts;
    uint32_t enqueued_frames;
    uint32_t delivered_frames;
    uint32_t dropped_frames;
    uint32_t duplicated_frames;
    uint32_t reordered_frames;
    uint32_t outage_drops;
    uint32_t queue_overflows;
} SimSwarmNetworkStats;

typedef struct {
    uint8_t data[SIM_SWARM_NETWORK_MAX_FRAME_SIZE];
    uint32_t deliver_at_ms;
    uint32_t insertion_order;
    uint16_t size;
    uint8_t source_id;
    uint8_t destination_id;
    uint8_t occupied;
} SimSwarmNetworkPacket;

typedef struct {
    SimSwarmNetworkConfig cfg;
    SimSwarmNetworkStats stats;
    SimSwarmNetworkPacket queue[SIM_SWARM_NETWORK_QUEUE_CAPACITY];
    uint8_t node_ids[SIM_SWARM_NETWORK_MAX_NODES];
    uint8_t node_count;
    uint32_t rng_state;
    uint32_t insertion_sequence;
    uint32_t broadcast_sequence;
    uint32_t node_broadcast_sequences[SIM_SWARM_NETWORK_MAX_NODES];
} SimSwarmNetwork;

void sim_swarm_network_config_default(SimSwarmNetworkConfig *cfg);
void sim_swarm_network_init(SimSwarmNetwork *network,
                            const SimSwarmNetworkConfig *cfg);
SimSwarmNetworkStatus sim_swarm_network_add_node(SimSwarmNetwork *network,
                                                  uint8_t agent_id);

/* Broadcasts one frame to every registered node except the source. */
SimSwarmNetworkStatus sim_swarm_network_broadcast(
    SimSwarmNetwork *network,
    uint8_t source_id,
    const uint8_t *frame,
    uint16_t size,
    uint32_t now_ms);

/* Returns 1 and removes one due frame, 0 when none is ready, -1 on error. */
int sim_swarm_network_receive(SimSwarmNetwork *network,
                              uint8_t destination_id,
                              uint32_t now_ms,
                              uint8_t *frame,
                              uint16_t capacity,
                              uint16_t *size,
                              uint8_t *source_id);

uint8_t sim_swarm_network_pending(const SimSwarmNetwork *network);

#ifdef __cplusplus
}
#endif

#endif /* SIM_SWARM_NETWORK_H */
