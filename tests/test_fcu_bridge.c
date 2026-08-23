#include <math.h>
#include <stdio.h>
#include <string.h>

#include "fcu_bridge.h"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; \
} } while (0)

static uint8_t nearf(float a, float b)
{
    return fabsf(a - b) <= 1e-5f ? 1u : 0u;
}

static FcuStateSnapshot make_state(uint16_t sequence)
{
    FcuStateSnapshot state;
    memset(&state, 0, sizeof(state));
    state.sequence = sequence;
    state.source_timestamp_ms = 1234u;
    state.valid_mask = FCU_STATE_VALID_KNOWN_MASK;
    state.capabilities = FCU_CAP_VELOCITY_SETPOINT |
        FCU_CAP_ACCELERATION_SETPOINT | FCU_CAP_YAW_SETPOINT |
        FCU_CAP_EXTERNAL_CONTROL;
    state.attitude = quat_identity();
    state.angular_velocity = vec3(0.1f, -0.2f, 0.3f);
    state.linear_acceleration = vec3(0.0f, 0.0f, NAV_GRAVITY);
    state.local_position = vec3(1.0f, 2.0f, 3.0f);
    state.local_velocity = vec3(0.4f, 0.5f, 0.6f);
    state.height_m = 2.8f;
    state.battery_remaining = 0.75f;
    state.armed = 1u;
    state.external_control_active = 1u;
    state.estimator_healthy = 1u;
    return state;
}

static void make_nav_command(GuidanceOutput *guidance, CtrlOutput *control)
{
    guidance->pos_sp = vec3(2.0f, 3.0f, 1.0f);
    guidance->vel_sp = vec3(0.8f, -0.2f, 0.1f);
    guidance->accel_sp = vec3_zero();
    guidance->yaw_sp = 0.7f;
    guidance->use_pos_sp = 1u;
    guidance->use_accel_sp = 0u;
    control->accel_cmd = vec3(0.3f, -0.1f, 0.2f);
    control->yaw_rate_cmd = -0.4f;
}

int main(void)
{
    FcuBridgeConfig cfg;
    FcuBridge bridge;
    FcuStateSnapshot state;
    FcuStateSnapshot decoded_state;
    FcuSetpoint setpoint;
    FcuSetpoint decoded_setpoint;
    GuidanceOutput guidance;
    CtrlOutput control;
    ImuSample imu;
    OdomSample odometry;
    FcuLinkParser parser;
    FcuLinkFrame parsed;
    uint8_t state_frame[FCU_LINK_STATE_FRAME_SIZE];
    uint8_t command_frame[FCU_LINK_SETPOINT_FRAME_SIZE];
    uint32_t age_ms = 0u;
    float height_m = -1.0f;
    uint16_t index;
    uint8_t frame_seen = 0u;

    fcu_bridge_config_default(&cfg);
    CHECK(fcu_bridge_config_valid(&cfg));
    cfg.state_timeout_ms = 0u;
    CHECK(!fcu_bridge_config_valid(&cfg));
    fcu_bridge_config_default(&cfg);

    state = make_state(7u);
    CHECK(fcu_link_encode_state(&state, state_frame) == FCU_LINK_OK);
    CHECK(fcu_link_decode_state(state_frame, sizeof(state_frame),
                                &decoded_state) == FCU_LINK_OK);
    CHECK(decoded_state.sequence == state.sequence);
    CHECK(decoded_state.valid_mask == state.valid_mask);
    CHECK(decoded_state.capabilities == state.capabilities);
    CHECK(nearf(decoded_state.local_position.z, 3.0f));
    CHECK(nearf(decoded_state.battery_remaining, 0.75f));
    CHECK(decoded_state.armed && decoded_state.external_control_active);

    state_frame[64] ^= 0x20u;
    CHECK(fcu_link_decode_state(state_frame, sizeof(state_frame),
                                &decoded_state) == FCU_LINK_ERROR_CRC);
    CHECK(fcu_link_encode_state(&state, state_frame) == FCU_LINK_OK);

    fcu_link_parser_init(&parser);
    CHECK(fcu_link_parser_push_byte(&parser, 0xa5u, &parsed) ==
          FCU_LINK_PARSE_REJECTED);
    for (index = 0u; index < FCU_LINK_STATE_FRAME_SIZE; index++) {
        FcuLinkParseResult result =
            fcu_link_parser_push_byte(&parser, state_frame[index], &parsed);
        if (result == FCU_LINK_PARSE_FRAME) frame_seen = 1u;
    }
    CHECK(frame_seen);
    CHECK(parsed.size == FCU_LINK_STATE_FRAME_SIZE);
    CHECK(parser.completed_frames == 1u);
    CHECK(fcu_link_decode_state(parsed.data, parsed.size, &decoded_state) ==
          FCU_LINK_OK);

    fcu_bridge_init(&bridge, &cfg);
    CHECK(fcu_bridge_update(&bridge, 0u) == FCU_BRIDGE_WAITING);
    CHECK(fcu_bridge_ingest_state(&bridge, &state, 100u) ==
          FCU_BRIDGE_ACCEPTED);
    CHECK(fcu_bridge_ingest_state(&bridge, &state, 101u) ==
          FCU_BRIDGE_REJECTED_DUPLICATE);
    decoded_state = state;
    decoded_state.sequence = 6u;
    CHECK(fcu_bridge_ingest_state(&bridge, &decoded_state, 102u) ==
          FCU_BRIDGE_REJECTED_OUT_OF_ORDER);
    CHECK(fcu_bridge_get_state(&bridge, 120u, &decoded_state, &age_ms));
    CHECK(age_ms == 20u);
    CHECK(fcu_bridge_update(&bridge, 151u) == FCU_BRIDGE_STALE);
    CHECK(bridge.stats.stale_transitions == 1u);
    decoded_state = state;
    decoded_state.sequence = 0u;
    decoded_state.source_timestamp_ms = 5u;
    CHECK(fcu_bridge_ingest_state(&bridge, &decoded_state, 160u) ==
          FCU_BRIDGE_RESYNCHRONIZED);
    CHECK(bridge.stats.resynchronized_states == 1u);

    make_nav_command(&guidance, &control);
    CHECK(fcu_bridge_make_setpoint(&bridge, &guidance, &control, 1u, 170u,
                                   &setpoint));
    CHECK(setpoint.preferred_control == FCU_CONTROL_VELOCITY);
    CHECK((setpoint.valid_mask & FCU_SETPOINT_VALID_VELOCITY) != 0u);
    CHECK((setpoint.valid_mask & FCU_SETPOINT_VALID_ACCELERATION) == 0u);
    CHECK((setpoint.valid_mask & FCU_SETPOINT_VALID_YAW) != 0u);
    CHECK((setpoint.valid_mask & FCU_SETPOINT_VALID_YAW_RATE) == 0u);
    CHECK(fcu_link_encode_setpoint(&setpoint, command_frame) == FCU_LINK_OK);
    CHECK(fcu_link_decode_setpoint(command_frame, sizeof(command_frame),
                                   &decoded_setpoint) == FCU_LINK_OK);
    CHECK(nearf(decoded_setpoint.velocity_sp.x, 0.8f));
    CHECK(fcu_setpoint_is_fresh(&decoded_setpoint, 180u, 280u));
    CHECK(!fcu_setpoint_is_fresh(&decoded_setpoint, 180u, 281u));
    command_frame[40] ^= 0x01u;
    CHECK(fcu_link_decode_setpoint(command_frame, sizeof(command_frame),
                                   &decoded_setpoint) == FCU_LINK_ERROR_CRC);

    CHECK(fcu_state_to_nav_samples(&state, 500u, &imu, &odometry,
                                   &height_m));
    CHECK(imu.timestamp_ms == 500u);
    CHECK(nearf(imu.gyro.z, 0.3f));
    CHECK(odometry.valid == 1u);
    CHECK(nearf(odometry.pos.y, 2.0f));
    CHECK(nearf(height_m, 2.8f));

    /* Auto mode falls back to acceleration when velocity is unsupported. */
    fcu_bridge_init(&bridge, &cfg);
    state = make_state(65535u);
    state.capabilities = FCU_CAP_ACCELERATION_SETPOINT |
        FCU_CAP_YAW_RATE_SETPOINT | FCU_CAP_EXTERNAL_CONTROL;
    CHECK(fcu_bridge_ingest_state(&bridge, &state, 1000u) ==
          FCU_BRIDGE_ACCEPTED);
    CHECK(fcu_bridge_make_setpoint(&bridge, &guidance, &control, 1u, 1010u,
                                   &setpoint));
    CHECK(setpoint.preferred_control == FCU_CONTROL_ACCELERATION);
    CHECK((setpoint.valid_mask & FCU_SETPOINT_VALID_ACCELERATION) != 0u);
    CHECK((setpoint.valid_mask & FCU_SETPOINT_VALID_VELOCITY) == 0u);
    CHECK((setpoint.valid_mask & FCU_SETPOINT_VALID_YAW_RATE) != 0u);
    state.sequence = 0u;
    CHECK(fcu_bridge_ingest_state(&bridge, &state, 1020u) ==
          FCU_BRIDGE_ACCEPTED);

    state.linear_acceleration.x = NAN;
    CHECK(fcu_bridge_ingest_state(&bridge, &state, 1030u) ==
          FCU_BRIDGE_REJECTED_INVALID);

    fcu_setpoint_disable(9u, 2000u, &setpoint);
    CHECK(!setpoint.motion_enabled);
    CHECK(setpoint.valid_mask == 0u);
    CHECK(fcu_link_encode_setpoint(&setpoint, command_frame) == FCU_LINK_OK);
    CHECK(!fcu_setpoint_is_fresh(&setpoint, 2000u, 2000u));

    if (failures == 0) {
        printf("test_fcu_bridge: PASS\n");
        return 0;
    }
    printf("test_fcu_bridge: %d FAILURES\n", failures);
    return 1;
}
