/* Versioned, endian-stable navigation telemetry frames. */
#ifndef NAV_TELEMETRY_H
#define NAV_TELEMETRY_H

#include <stdint.h>

#include "nav_runtime.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NAV_TELEMETRY_SCHEMA_VERSION 1u
#define NAV_TELEMETRY_FRAME_SIZE 152u

enum {
    NAV_TELEMETRY_FLAG_ARMED = 1u << 0,
    NAV_TELEMETRY_FLAG_STEP_VALID = 1u << 1,
    NAV_TELEMETRY_FLAG_TRAJECTORY_VALID = 1u << 2,
    NAV_TELEMETRY_FLAG_TARGET_VISIBLE = 1u << 3,
    NAV_TELEMETRY_FLAG_HOME_VISIBLE = 1u << 4,
    NAV_TELEMETRY_FLAG_WATCHDOG_TRIPPED = 1u << 5,
    NAV_TELEMETRY_FLAG_RECOVERY_ACTIVE = 1u << 6,
    NAV_TELEMETRY_FLAG_MISSION_COMPLETE = 1u << 7,
    NAV_TELEMETRY_FLAG_MISSION_FAILED = 1u << 8
};

typedef enum {
    NAV_TELEMETRY_OK = 0,
    NAV_TELEMETRY_ERROR_ARGUMENT,
    NAV_TELEMETRY_ERROR_MAGIC,
    NAV_TELEMETRY_ERROR_VERSION,
    NAV_TELEMETRY_ERROR_SIZE,
    NAV_TELEMETRY_ERROR_CRC,
    NAV_TELEMETRY_ERROR_VALUE
} NavTelemetryStatus;

typedef struct {
    uint32_t sequence;
    uint32_t timestamp_ms;
    uint32_t event_flags;
    uint32_t safety_reason_mask;
    uint32_t stale_source_mask;
    uint32_t duplicate_source_mask;
    uint32_t out_of_order_source_mask;
    uint32_t nonfinite_source_mask;
    uint32_t invalid_source_mask;
    uint32_t cycle_gap_ms;
    uint32_t flags;
    Vec3f position;
    Vec3f velocity;
    float yaw;
    float estimator_quality;
    Vec3f target_relative_position;
    Vec3f home_relative_position;
    Vec3f guidance_velocity;
    Vec3f acceleration_command;
    float yaw_rate_command;
    float remaining_mission_s;
    uint8_t mission_state;
    uint8_t transition_reason;
    uint8_t estimator_status;
    uint8_t safety_level;
    uint8_t terminal_stage;
    uint8_t recovery_stage;
    uint8_t target_status;
    uint8_t docking_stage;
} NavTelemetrySnapshot;

uint32_t nav_telemetry_crc32(const uint8_t *data, uint32_t size);

/* Returns 1 only when all fields required by schema v1 are finite and valid. */
uint8_t nav_telemetry_capture(NavTelemetrySnapshot *snapshot,
                              const NavRuntimeOutput *output,
                              uint32_t sequence, uint32_t timestamp_ms);

NavTelemetryStatus nav_telemetry_encode(
    const NavTelemetrySnapshot *snapshot,
    uint8_t frame[NAV_TELEMETRY_FRAME_SIZE]);
NavTelemetryStatus nav_telemetry_decode(
    const uint8_t *frame, uint16_t frame_size,
    NavTelemetrySnapshot *snapshot);
const char *nav_telemetry_status_name(NavTelemetryStatus status);

#ifdef __cplusplus
}
#endif

#endif /* NAV_TELEMETRY_H */
