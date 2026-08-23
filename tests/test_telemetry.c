#include <math.h>
#include <stdio.h>
#include <string.h>

#include "nav_telemetry.h"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; \
} } while (0)

int main(void)
{
    static const uint8_t crc_text[] = "123456789";
    NavRuntimeOutput output;
    NavTelemetrySnapshot snapshot;
    NavTelemetrySnapshot decoded;
    uint8_t frame[NAV_TELEMETRY_FRAME_SIZE];
    uint8_t second_frame[NAV_TELEMETRY_FRAME_SIZE];

    CHECK(nav_telemetry_crc32(crc_text, 9u) == 0xcbf43926u);
    memset(&output, 0, sizeof(output));
    output.nav.pos = vec3(1.25f, -2.5f, 0.75f);
    output.nav.vel = vec3(0.1f, 0.2f, -0.3f);
    output.nav.yaw = 0.4f;
    output.nav.quality = 0.85f;
    output.nav.status = EST_TRACKING;
    output.mission.state = MS_RETURN_HOME;
    output.mission.transition_reason = MISSION_REASON_RECOVERY_COMPLETE;
    output.mission.docking_stage = DOCK_APPROACH;
    output.safety.level = SAFETY_RETURN_REQUIRED;
    output.safety.reason_mask = SAFETY_REASON_SOFT_DEADLINE |
                                SAFETY_REASON_SWARM_LINK;
    output.safety.remaining_s = 12.5f;
    output.health.stale_source_mask = NAV_INPUT_SOURCE_FLOW;
    output.health.cycle_gap_ms = 10u;
    output.target.status = TARGET_TRACK_COASTING;
    output.target.rel_pos = vec3(2.0f, 1.0f, 0.5f);
    output.home.rel_pos = vec3(-1.0f, 0.25f, -0.75f);
    output.guidance.vel_sp = vec3(0.4f, -0.2f, 0.1f);
    output.control.accel_cmd = vec3(0.5f, 0.6f, -0.7f);
    output.control.yaw_rate_cmd = -0.8f;
    output.terminal_stage = TERMINAL_CLOSING;
    output.recovery.stage = RECOVERY_BREAKAWAY;
    output.event_flags = NAV_EVENT_RELOCALIZED;
    output.armed = 1u;
    output.step_valid = 1u;
    output.trajectory_valid = 1u;
    output.target.visible = 1u;

    CHECK(nav_telemetry_capture(&snapshot, &output, 0x12345678u,
                                0x89abcdefu) == 1u);
    CHECK((snapshot.flags & NAV_TELEMETRY_FLAG_ARMED) != 0u);
    CHECK((snapshot.flags & NAV_TELEMETRY_FLAG_TARGET_VISIBLE) != 0u);
    CHECK(nav_telemetry_encode(&snapshot, frame) == NAV_TELEMETRY_OK);
    CHECK(nav_telemetry_encode(&snapshot, second_frame) == NAV_TELEMETRY_OK);
    CHECK(memcmp(frame, second_frame, sizeof(frame)) == 0);
    CHECK(frame[0] == 'R' && frame[1] == 'M' &&
          frame[2] == 'N' && frame[3] == 'T');
    CHECK(frame[8] == 0x78u && frame[9] == 0x56u &&
          frame[10] == 0x34u && frame[11] == 0x12u);
    CHECK(nav_telemetry_decode(frame, NAV_TELEMETRY_FRAME_SIZE, &decoded) ==
          NAV_TELEMETRY_OK);
    CHECK(decoded.sequence == snapshot.sequence);
    CHECK(decoded.timestamp_ms == snapshot.timestamp_ms);
    CHECK(decoded.mission_state == (uint8_t)MS_RETURN_HOME);
    CHECK(decoded.estimator_status == (uint8_t)EST_TRACKING);
    CHECK(decoded.safety_reason_mask == snapshot.safety_reason_mask);
    CHECK(decoded.position.x == snapshot.position.x);
    CHECK(decoded.acceleration_command.z == snapshot.acceleration_command.z);
    CHECK(decoded.flags == snapshot.flags);

    memcpy(second_frame, frame, sizeof(frame));
    second_frame[0] = 'X';
    CHECK(nav_telemetry_decode(second_frame, NAV_TELEMETRY_FRAME_SIZE,
                               &decoded) ==
          NAV_TELEMETRY_ERROR_MAGIC);
    memcpy(second_frame, frame, sizeof(frame));
    second_frame[4] = (uint8_t)(NAV_TELEMETRY_SCHEMA_VERSION + 1u);
    CHECK(nav_telemetry_decode(second_frame, NAV_TELEMETRY_FRAME_SIZE,
                               &decoded) ==
          NAV_TELEMETRY_ERROR_VERSION);
    memcpy(second_frame, frame, sizeof(frame));
    second_frame[6] = 0u;
    second_frame[7] = 0u;
    CHECK(nav_telemetry_decode(second_frame, NAV_TELEMETRY_FRAME_SIZE,
                               &decoded) ==
          NAV_TELEMETRY_ERROR_SIZE);
    memcpy(second_frame, frame, sizeof(frame));
    second_frame[60] ^= 0x01u;
    CHECK(nav_telemetry_decode(second_frame, NAV_TELEMETRY_FRAME_SIZE,
                               &decoded) ==
          NAV_TELEMETRY_ERROR_CRC);

    output.nav.pos.x = NAN;
    CHECK(nav_telemetry_capture(&snapshot, &output, 1u, 1u) == 0u);
    CHECK(nav_telemetry_encode(0, frame) == NAV_TELEMETRY_ERROR_ARGUMENT);
    CHECK(nav_telemetry_decode(frame, NAV_TELEMETRY_FRAME_SIZE - 1u,
                               &decoded) == NAV_TELEMETRY_ERROR_SIZE);
    CHECK(nav_telemetry_decode(0, NAV_TELEMETRY_FRAME_SIZE, &decoded) ==
          NAV_TELEMETRY_ERROR_ARGUMENT);

    if (failures == 0) printf("telemetry tests passed\n");
    return failures == 0 ? 0 : 1;
}
