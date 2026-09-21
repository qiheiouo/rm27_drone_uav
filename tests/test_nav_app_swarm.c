#include <stdio.h>
#include <string.h>

#include "nav_tasks.h"
#include "nav_platform.h"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; \
} } while (0)

#define RX_FRAME_COUNT 5u

static uint32_t fake_time_ms;
static uint8_t rx_frames[RX_FRAME_COUNT][SWARM_LINK_STATE_FRAME_SIZE];
static uint16_t rx_sizes[RX_FRAME_COUNT];
static uint8_t rx_index;
static uint8_t tx_frame[SWARM_LINK_STATE_FRAME_SIZE];
static uint16_t tx_size;
static uint32_t tx_count;
static uint8_t tx_reject;

static AgentState make_peer(uint8_t id, uint16_t sequence, float x)
{
    AgentState state;
    agent_state_clear(&state);
    state.agent_id = id;
    state.sequence = sequence;
    state.timestamp_ms = 100u + id;
    state.pos = vec3(x, 0.0f, 1.0f);
    state.vel = vec3_zero();
    state.quality = 0.9f;
    state.valid = 1u;
    return state;
}

uint32_t nav_time_ms(void)
{
    return fake_time_ms;
}

int nav_imu_read(ImuSample *out)
{
    out->accel = vec3(0.0f, 0.0f, NAV_GRAVITY);
    out->gyro = vec3_zero();
    out->timestamp_ms = fake_time_ms;
    return 0;
}

int nav_tof_read(float *height_m)
{
    *height_m = 0.0f;
    return 0;
}

int nav_flow_read(FlowFrame *out)
{
    (void)out;
    return -1;
}

int nav_camera_target_read(PixelObs *out)
{
    (void)out;
    return -1;
}

int nav_camera_home_read(PixelObs *out)
{
    (void)out;
    return -1;
}

int nav_local_obstacles_read(DynamicObstacleSet *out)
{
    (void)out;
    return -1;
}

int nav_mission_command_read(NavMissionCommand *out)
{
    (void)out;
    return -1;
}

int nav_dock_status_read(NavDockStatus *out)
{
    (void)out;
    return -1;
}

int nav_swarm_frame_read(uint8_t *frame, uint16_t capacity, uint16_t *size)
{
    if (rx_index >= RX_FRAME_COUNT) return -1;
    if (capacity < rx_sizes[rx_index]) return -2;
    memcpy(frame, rx_frames[rx_index], rx_sizes[rx_index]);
    *size = rx_sizes[rx_index];
    rx_index++;
    return 0;
}

int nav_swarm_frame_write(const uint8_t *frame, uint16_t size)
{
    if (tx_reject) return -1;
    if (size > sizeof(tx_frame)) return -2;
    memcpy(tx_frame, frame, size);
    tx_size = size;
    tx_count++;
    return 0;
}

int nav_fcu_setpoint_write(const FcuSetpoint *setpoint)
{
    (void)setpoint;
    return 0;
}

void nav_fcu_set_armed(uint8_t armed)
{
    (void)armed;
}

void nav_log(const char *msg)
{
    (void)msg;
}

int nav_telemetry_write(const uint8_t *frame, uint16_t size)
{
    (void)frame;
    (void)size;
    return 0;
}

int main(void)
{
    NavApp app;
    NavAppConfig cfg;
    AgentState state;
    AgentState outgoing;
    uint8_t step;

    nav_runtime_config_default(&cfg);
    cfg.swarm_avoidance.mode = SWARM_ENABLED;
    state = make_peer(1u, 1u, 10.0f);
    CHECK(swarm_link_encode_state(&state, rx_frames[0]) == SWARM_LINK_OK);
    memcpy(rx_frames[1], rx_frames[0], SWARM_LINK_STATE_FRAME_SIZE);
    state = make_peer(3u, 1u, 11.0f);
    CHECK(swarm_link_encode_state(&state, rx_frames[2]) == SWARM_LINK_OK);
    rx_frames[2][20] ^= 0x80u;
    CHECK(swarm_link_encode_state(&state, rx_frames[3]) == SWARM_LINK_OK);
    state = make_peer(4u, 1u, 12.0f);
    CHECK(swarm_link_encode_state(&state, rx_frames[4]) == SWARM_LINK_OK);
    for (step = 0u; step < RX_FRAME_COUNT; step++) {
        rx_sizes[step] = SWARM_LINK_STATE_FRAME_SIZE;
    }

    CHECK(nav_app_init(&app, &cfg) == NAV_CONFIG_ERROR_NONE);
    fake_time_ms = 10u;
    nav_app_step(&app, cfg.nominal_dt_s);
    CHECK(rx_index == NAV_APP_SWARM_RX_BUDGET);
    CHECK(app.swarm_peers.stats.accepted_frames == 2u);
    CHECK(app.swarm_peers.stats.duplicate_frames == 1u);
    CHECK(app.swarm_decode_errors == 1u);
    CHECK(app.runtime.swarm.other_count == 2u);

    fake_time_ms = 20u;
    nav_app_step(&app, cfg.nominal_dt_s);
    CHECK(rx_index == RX_FRAME_COUNT);
    CHECK(app.swarm_peers.stats.accepted_frames == 3u);
    CHECK(app.runtime.swarm.other_count == 3u);

    for (step = 3u; step <= NAV_APP_SWARM_TX_PERIOD_STEPS; step++) {
        fake_time_ms = (uint32_t)step * 10u;
        nav_app_step(&app, cfg.nominal_dt_s);
    }
    CHECK(tx_count == 1u);
    CHECK(tx_size == SWARM_LINK_STATE_FRAME_SIZE);
    CHECK(swarm_link_decode_state(tx_frame, tx_size, &outgoing) ==
          SWARM_LINK_OK);
    CHECK(outgoing.agent_id == cfg.self_agent_id);
    CHECK(outgoing.sequence == 0u);

    tx_reject = 1u;
    for (step = 1u; step <= NAV_APP_SWARM_TX_PERIOD_STEPS; step++) {
        fake_time_ms += 10u;
        nav_app_step(&app, cfg.nominal_dt_s);
    }
    CHECK(tx_count == 1u);
    CHECK(app.swarm_tx_drops == 1u);
    CHECK(app.swarm_tx_sequence == 2u);

    nav_runtime_config_default(&cfg);
    rx_index = 0u;
    tx_count = 0u;
    tx_reject = 0u;
    CHECK(nav_app_init(&app, &cfg) == NAV_CONFIG_ERROR_NONE);
    for (step = 1u; step <= NAV_APP_SWARM_TX_PERIOD_STEPS; step++) {
        fake_time_ms = 1000u + (uint32_t)step * 10u;
        nav_app_step(&app, cfg.nominal_dt_s);
    }
    CHECK(rx_index == 0u);
    CHECK(tx_count == 0u);

    nav_runtime_config_default(&cfg);
    cfg.swarm_collision.max_message_age_s = -1.0f;
    CHECK((nav_app_init(&app, &cfg) & NAV_CONFIG_ERROR_SWARM) != 0u);

    if (failures == 0) {
        printf("test_nav_app_swarm: PASS\n");
        return 0;
    }
    printf("test_nav_app_swarm: %d FAILURES\n", failures);
    return 1;
}
