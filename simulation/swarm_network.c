#include <string.h>

#include "swarm_network.h"

static uint8_t node_registered(const SimSwarmNetwork *network, uint8_t agent_id)
{
    uint8_t index;
    for (index = 0u; index < network->node_count; index++) {
        if (network->node_ids[index] == agent_id) {
            return 1u;
        }
    }
    return 0u;
}

static int node_index(const SimSwarmNetwork *network, uint8_t agent_id)
{
    uint8_t index;
    for (index = 0u; index < network->node_count; index++) {
        if (network->node_ids[index] == agent_id) return (int)index;
    }
    return -1;
}

static uint8_t time_reached(uint32_t now_ms, uint32_t target_ms)
{
    return ((int32_t)(now_ms - target_ms) >= 0) ? 1u : 0u;
}

static uint8_t outage_active(const SimSwarmNetworkConfig *cfg,
                             uint32_t now_ms)
{
    if (cfg->outage_start_ms >= cfg->outage_end_ms) {
        return 0u;
    }
    return (now_ms >= cfg->outage_start_ms &&
            now_ms < cfg->outage_end_ms) ? 1u : 0u;
}

static uint32_t next_random(SimSwarmNetwork *network)
{
    network->rng_state = network->rng_state * 1664525u + 1013904223u;
    return network->rng_state;
}

static uint32_t delivery_delay(SimSwarmNetwork *network,
                               uint8_t reordered)
{
    uint32_t delay = network->cfg.base_delay_ms;
    if (network->cfg.jitter_ms > 0u) {
        uint32_t random_value = next_random(network);
        delay += network->cfg.jitter_ms == UINT32_MAX
            ? random_value
            : random_value % (network->cfg.jitter_ms + 1u);
    }
    if (reordered) {
        delay += network->cfg.reorder_extra_delay_ms;
    }
    return delay;
}

static SimSwarmNetworkStatus enqueue(SimSwarmNetwork *network,
                                     uint8_t source_id,
                                     uint8_t destination_id,
                                     const uint8_t *frame,
                                     uint16_t size,
                                     uint32_t deliver_at_ms)
{
    uint8_t index;
    for (index = 0u; index < SIM_SWARM_NETWORK_QUEUE_CAPACITY; index++) {
        SimSwarmNetworkPacket *packet = &network->queue[index];
        if (!packet->occupied) {
            memcpy(packet->data, frame, size);
            packet->deliver_at_ms = deliver_at_ms;
            packet->insertion_order = network->insertion_sequence++;
            packet->size = size;
            packet->source_id = source_id;
            packet->destination_id = destination_id;
            packet->occupied = 1u;
            network->stats.enqueued_frames++;
            return SIM_SWARM_NETWORK_OK;
        }
    }
    network->stats.queue_overflows++;
    network->stats.dropped_frames++;
    return SIM_SWARM_NETWORK_ERROR_CAPACITY;
}

void sim_swarm_network_config_default(SimSwarmNetworkConfig *cfg)
{
    if (cfg == 0) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->base_delay_ms = 20u;
    cfg->seed = 1u;
}

void sim_swarm_network_init(SimSwarmNetwork *network,
                            const SimSwarmNetworkConfig *cfg)
{
    SimSwarmNetworkConfig defaults;
    if (network == 0) return;
    sim_swarm_network_config_default(&defaults);
    memset(network, 0, sizeof(*network));
    network->cfg = cfg != 0 ? *cfg : defaults;
    network->rng_state = network->cfg.seed != 0u ? network->cfg.seed : 1u;
}

SimSwarmNetworkStatus sim_swarm_network_add_node(SimSwarmNetwork *network,
                                                  uint8_t agent_id)
{
    if (network == 0 || agent_id == 0u) {
        return SIM_SWARM_NETWORK_ERROR_ARGUMENT;
    }
    if (node_registered(network, agent_id)) {
        return SIM_SWARM_NETWORK_ERROR_NODE;
    }
    if (network->node_count >= SIM_SWARM_NETWORK_MAX_NODES) {
        return SIM_SWARM_NETWORK_ERROR_CAPACITY;
    }
    network->node_ids[network->node_count++] = agent_id;
    return SIM_SWARM_NETWORK_OK;
}

SimSwarmNetworkStatus sim_swarm_network_broadcast(
    SimSwarmNetwork *network,
    uint8_t source_id,
    const uint8_t *frame,
    uint16_t size,
    uint32_t now_ms)
{
    uint32_t broadcast_index;
    int source_index;
    uint8_t drop;
    uint8_t duplicate;
    uint8_t reordered;
    uint8_t destination_index;
    SimSwarmNetworkStatus result = SIM_SWARM_NETWORK_OK;

    if (network == 0 || frame == 0) {
        return SIM_SWARM_NETWORK_ERROR_ARGUMENT;
    }
    if (size == 0u || size > SIM_SWARM_NETWORK_MAX_FRAME_SIZE) {
        return SIM_SWARM_NETWORK_ERROR_FRAME;
    }
    source_index = node_index(network, source_id);
    if (source_index < 0) {
        return SIM_SWARM_NETWORK_ERROR_NODE;
    }

    network->stats.broadcasts++;
    network->broadcast_sequence++;
    broadcast_index =
        ++network->node_broadcast_sequences[(uint8_t)source_index];
    drop = (network->cfg.drop_every_n > 0u &&
            broadcast_index % network->cfg.drop_every_n == 0u) ? 1u : 0u;
    duplicate = (network->cfg.duplicate_every_n > 0u &&
                 broadcast_index % network->cfg.duplicate_every_n == 0u) ? 1u : 0u;
    reordered = (network->cfg.reorder_every_n > 0u &&
                 broadcast_index % network->cfg.reorder_every_n == 0u) ? 1u : 0u;

    for (destination_index = 0u; destination_index < network->node_count;
         destination_index++) {
        uint8_t destination_id = network->node_ids[destination_index];
        uint32_t delay;
        if (destination_id == source_id) continue;
        network->stats.delivery_attempts++;
        if (outage_active(&network->cfg, now_ms)) {
            network->stats.outage_drops++;
            network->stats.dropped_frames++;
            continue;
        }
        if (drop) {
            network->stats.dropped_frames++;
            continue;
        }
        delay = delivery_delay(network, reordered);
        if (reordered) network->stats.reordered_frames++;
        if (enqueue(network, source_id, destination_id, frame, size,
                    now_ms + delay) != SIM_SWARM_NETWORK_OK) {
            result = SIM_SWARM_NETWORK_ERROR_CAPACITY;
            continue;
        }
        if (duplicate) {
            network->stats.duplicated_frames++;
            if (enqueue(network, source_id, destination_id, frame, size,
                        now_ms + delay + 1u) != SIM_SWARM_NETWORK_OK) {
                result = SIM_SWARM_NETWORK_ERROR_CAPACITY;
            }
        }
    }
    return result;
}

int sim_swarm_network_receive(SimSwarmNetwork *network,
                              uint8_t destination_id,
                              uint32_t now_ms,
                              uint8_t *frame,
                              uint16_t capacity,
                              uint16_t *size,
                              uint8_t *source_id)
{
    int selected = -1;
    uint8_t index;
    if (network == 0 || frame == 0 || size == 0 || source_id == 0 ||
        !node_registered(network, destination_id)) {
        return -1;
    }
    for (index = 0u; index < SIM_SWARM_NETWORK_QUEUE_CAPACITY; index++) {
        const SimSwarmNetworkPacket *packet = &network->queue[index];
        if (!packet->occupied || packet->destination_id != destination_id ||
            !time_reached(now_ms, packet->deliver_at_ms)) {
            continue;
        }
        if (selected < 0 ||
            packet->insertion_order <
                network->queue[(uint8_t)selected].insertion_order) {
            selected = (int)index;
        }
    }
    if (selected < 0) return 0;
    if (network->queue[(uint8_t)selected].size > capacity) return -1;

    memcpy(frame, network->queue[(uint8_t)selected].data,
           network->queue[(uint8_t)selected].size);
    *size = network->queue[(uint8_t)selected].size;
    *source_id = network->queue[(uint8_t)selected].source_id;
    network->queue[(uint8_t)selected].occupied = 0u;
    network->stats.delivered_frames++;
    return 1;
}

uint8_t sim_swarm_network_pending(const SimSwarmNetwork *network)
{
    uint8_t count = 0u;
    uint8_t index;
    if (network == 0) return 0u;
    for (index = 0u; index < SIM_SWARM_NETWORK_QUEUE_CAPACITY; index++) {
        if (network->queue[index].occupied && count < 255u) count++;
    }
    return count;
}
