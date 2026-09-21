#include <stdio.h>
#include <string.h>

#include "swarm_network.h"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; \
} } while (0)

static void fill_frame(uint8_t *frame, uint8_t marker)
{
    uint8_t index;
    for (index = 0u; index < SWARM_LINK_STATE_FRAME_SIZE; index++) {
        frame[index] = (uint8_t)(marker + index);
    }
}

static void test_delay_and_broadcast(void)
{
    SimSwarmNetworkConfig cfg;
    SimSwarmNetwork network;
    uint8_t sent[SWARM_LINK_STATE_FRAME_SIZE];
    uint8_t received[SWARM_LINK_STATE_FRAME_SIZE];
    uint16_t size = 0u;
    uint8_t source = 0u;

    sim_swarm_network_config_default(&cfg);
    cfg.base_delay_ms = 30u;
    sim_swarm_network_init(&network, &cfg);
    CHECK(sim_swarm_network_add_node(&network, 1u) == SIM_SWARM_NETWORK_OK);
    CHECK(sim_swarm_network_add_node(&network, 2u) == SIM_SWARM_NETWORK_OK);
    CHECK(sim_swarm_network_add_node(&network, 3u) == SIM_SWARM_NETWORK_OK);
    CHECK(sim_swarm_network_add_node(&network, 3u) ==
          SIM_SWARM_NETWORK_ERROR_NODE);

    fill_frame(sent, 7u);
    CHECK(sim_swarm_network_broadcast(&network, 1u, sent, sizeof(sent), 100u) ==
          SIM_SWARM_NETWORK_OK);
    CHECK(network.stats.delivery_attempts == 2u);
    CHECK(sim_swarm_network_pending(&network) == 2u);
    CHECK(sim_swarm_network_receive(&network, 2u, 129u, received,
                                    sizeof(received), &size, &source) == 0);
    CHECK(sim_swarm_network_receive(&network, 2u, 130u, received,
                                    sizeof(received), &size, &source) == 1);
    CHECK(size == sizeof(sent));
    CHECK(source == 1u);
    CHECK(memcmp(sent, received, sizeof(sent)) == 0);
    CHECK(sim_swarm_network_receive(&network, 3u, 130u, received,
                                    sizeof(received), &size, &source) == 1);
    CHECK(network.stats.delivered_frames == 2u);
}

static void test_fault_patterns(void)
{
    SimSwarmNetworkConfig cfg;
    SimSwarmNetwork network;
    uint8_t frame[SWARM_LINK_STATE_FRAME_SIZE];
    uint8_t received[SWARM_LINK_STATE_FRAME_SIZE];
    uint16_t size;
    uint8_t source;
    int receive_count = 0;
    uint32_t time_ms;

    sim_swarm_network_config_default(&cfg);
    cfg.base_delay_ms = 5u;
    cfg.drop_every_n = 2u;
    cfg.duplicate_every_n = 3u;
    cfg.reorder_every_n = 3u;
    cfg.reorder_extra_delay_ms = 20u;
    cfg.outage_start_ms = 40u;
    cfg.outage_end_ms = 60u;
    sim_swarm_network_init(&network, &cfg);
    CHECK(sim_swarm_network_add_node(&network, 1u) == SIM_SWARM_NETWORK_OK);
    CHECK(sim_swarm_network_add_node(&network, 2u) == SIM_SWARM_NETWORK_OK);
    fill_frame(frame, 1u);

    CHECK(sim_swarm_network_broadcast(&network, 1u, frame, sizeof(frame), 0u) ==
          SIM_SWARM_NETWORK_OK);
    CHECK(sim_swarm_network_broadcast(&network, 1u, frame, sizeof(frame), 10u) ==
          SIM_SWARM_NETWORK_OK);
    CHECK(sim_swarm_network_broadcast(&network, 1u, frame, sizeof(frame), 20u) ==
          SIM_SWARM_NETWORK_OK);
    CHECK(sim_swarm_network_broadcast(&network, 1u, frame, sizeof(frame), 40u) ==
          SIM_SWARM_NETWORK_OK);

    for (time_ms = 0u; time_ms <= 80u; time_ms++) {
        int result;
        do {
            result = sim_swarm_network_receive(&network, 2u, time_ms, received,
                sizeof(received), &size, &source);
            if (result == 1) receive_count++;
        } while (result == 1);
    }
    CHECK(receive_count == 3);
    CHECK(network.stats.dropped_frames == 2u);
    CHECK(network.stats.outage_drops == 1u);
    CHECK(network.stats.duplicated_frames == 1u);
    CHECK(network.stats.reordered_frames == 1u);
}

static void test_clock_wrap(void)
{
    SimSwarmNetworkConfig cfg;
    SimSwarmNetwork network;
    uint8_t frame[SWARM_LINK_STATE_FRAME_SIZE];
    uint8_t received[SWARM_LINK_STATE_FRAME_SIZE];
    uint16_t size;
    uint8_t source;

    sim_swarm_network_config_default(&cfg);
    cfg.base_delay_ms = 32u;
    sim_swarm_network_init(&network, &cfg);
    CHECK(sim_swarm_network_add_node(&network, 1u) == SIM_SWARM_NETWORK_OK);
    CHECK(sim_swarm_network_add_node(&network, 2u) == SIM_SWARM_NETWORK_OK);
    fill_frame(frame, 3u);
    CHECK(sim_swarm_network_broadcast(&network, 1u, frame, sizeof(frame),
                                      0xfffffff0u) == SIM_SWARM_NETWORK_OK);
    CHECK(sim_swarm_network_receive(&network, 2u, 15u, received,
                                    sizeof(received), &size, &source) == 0);
    CHECK(sim_swarm_network_receive(&network, 2u, 16u, received,
                                    sizeof(received), &size, &source) == 1);
}

static void test_seed_reproducibility(void)
{
    SimSwarmNetworkConfig cfg;
    SimSwarmNetwork first;
    SimSwarmNetwork second;
    uint8_t frame[SWARM_LINK_STATE_FRAME_SIZE];
    uint8_t first_received[SWARM_LINK_STATE_FRAME_SIZE];
    uint8_t second_received[SWARM_LINK_STATE_FRAME_SIZE];
    uint16_t first_size;
    uint16_t second_size;
    uint8_t first_source;
    uint8_t second_source;
    uint32_t transmission;
    uint32_t time_ms;

    sim_swarm_network_config_default(&cfg);
    cfg.seed = 77u;
    cfg.base_delay_ms = 10u;
    cfg.jitter_ms = 30u;
    cfg.duplicate_every_n = 3u;
    cfg.reorder_every_n = 2u;
    cfg.reorder_extra_delay_ms = 40u;
    sim_swarm_network_init(&first, &cfg);
    sim_swarm_network_init(&second, &cfg);
    CHECK(sim_swarm_network_add_node(&first, 1u) == SIM_SWARM_NETWORK_OK);
    CHECK(sim_swarm_network_add_node(&first, 2u) == SIM_SWARM_NETWORK_OK);
    CHECK(sim_swarm_network_add_node(&second, 1u) == SIM_SWARM_NETWORK_OK);
    CHECK(sim_swarm_network_add_node(&second, 2u) == SIM_SWARM_NETWORK_OK);

    for (transmission = 0u; transmission < 8u; transmission++) {
        fill_frame(frame, (uint8_t)transmission);
        CHECK(sim_swarm_network_broadcast(&first, 1u, frame, sizeof(frame),
                                          transmission * 20u) ==
              SIM_SWARM_NETWORK_OK);
        CHECK(sim_swarm_network_broadcast(&second, 1u, frame, sizeof(frame),
                                          transmission * 20u) ==
              SIM_SWARM_NETWORK_OK);
    }
    for (time_ms = 0u; time_ms < 300u; time_ms++) {
        int first_result = sim_swarm_network_receive(&first, 2u, time_ms,
            first_received, sizeof(first_received), &first_size, &first_source);
        int second_result = sim_swarm_network_receive(&second, 2u, time_ms,
            second_received, sizeof(second_received), &second_size,
            &second_source);
        CHECK(first_result == second_result);
        if (first_result == 1 && second_result == 1) {
            CHECK(first_size == second_size);
            CHECK(first_source == second_source);
            CHECK(memcmp(first_received, second_received, first_size) == 0);
        }
    }
    CHECK(first.stats.delivered_frames == second.stats.delivered_frames);
    CHECK(sim_swarm_network_pending(&first) ==
          sim_swarm_network_pending(&second));
}

int main(void)
{
    test_delay_and_broadcast();
    test_fault_patterns();
    test_clock_wrap();
    test_seed_reproducibility();
    if (failures == 0) {
        printf("test_swarm_network: PASS\n");
        return 0;
    }
    printf("test_swarm_network: %d FAILURES\n", failures);
    return 1;
}
