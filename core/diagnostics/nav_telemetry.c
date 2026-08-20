#include <string.h>

#include "nav_telemetry.h"

#define TELEMETRY_CRC_OFFSET 148u

typedef char telemetry_requires_32_bit_float[(sizeof(float) == 4u) ? 1 : -1];

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

static uint8_t snapshot_valid(const NavTelemetrySnapshot *snapshot)
{
    if (snapshot == 0 || snapshot->mission_state > (uint8_t)MS_EMERGENCY_LAND ||
        snapshot->transition_reason > (uint8_t)MISSION_REASON_EMERGENCY ||
        snapshot->estimator_status > (uint8_t)EST_RELOCALIZED ||
        snapshot->safety_level > (uint8_t)SAFETY_EMERGENCY ||
        snapshot->terminal_stage > (uint8_t)TERMINAL_FAILED ||
        snapshot->recovery_stage > (uint8_t)RECOVERY_FAILED ||
        snapshot->target_status > (uint8_t)TARGET_TRACK_COASTING ||
        snapshot->docking_stage > (uint8_t)DOCK_DOCKED) {
        return 0u;
    }
    return (vec3_is_finite(snapshot->position) &&
            vec3_is_finite(snapshot->velocity) &&
            nav_isfinite(snapshot->yaw) &&
            nav_isfinite(snapshot->estimator_quality) &&
            vec3_is_finite(snapshot->target_relative_position) &&
            vec3_is_finite(snapshot->home_relative_position) &&
            vec3_is_finite(snapshot->guidance_velocity) &&
            vec3_is_finite(snapshot->acceleration_command) &&
            nav_isfinite(snapshot->yaw_rate_command) &&
            nav_isfinite(snapshot->remaining_mission_s)) ? 1u : 0u;
}

uint32_t nav_telemetry_crc32(const uint8_t *data, uint32_t size)
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

uint8_t nav_telemetry_capture(NavTelemetrySnapshot *snapshot,
                              const NavRuntimeOutput *output,
                              uint32_t sequence, uint32_t timestamp_ms)
{
    if (snapshot == 0 || output == 0) return 0u;
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->sequence = sequence;
    snapshot->timestamp_ms = timestamp_ms;
    snapshot->event_flags = output->event_flags;
    snapshot->safety_reason_mask = output->safety.reason_mask;
    snapshot->stale_source_mask = output->health.stale_source_mask;
    snapshot->duplicate_source_mask = output->health.duplicate_source_mask;
    snapshot->out_of_order_source_mask = output->health.out_of_order_source_mask;
    snapshot->nonfinite_source_mask = output->health.nonfinite_source_mask;
    snapshot->invalid_source_mask = output->health.invalid_source_mask;
    snapshot->cycle_gap_ms = output->health.cycle_gap_ms;
    snapshot->position = output->nav.pos;
    snapshot->velocity = output->nav.vel;
    snapshot->yaw = output->nav.yaw;
    snapshot->estimator_quality = output->nav.quality;
    snapshot->target_relative_position = output->target.rel_pos;
    snapshot->home_relative_position = output->home.rel_pos;
    snapshot->guidance_velocity = output->guidance.vel_sp;
    snapshot->acceleration_command = output->control.accel_cmd;
    snapshot->yaw_rate_command = output->control.yaw_rate_cmd;
    snapshot->remaining_mission_s = output->safety.remaining_s;
    snapshot->mission_state = (uint8_t)output->mission.state;
    snapshot->transition_reason = (uint8_t)output->mission.transition_reason;
    snapshot->estimator_status = (uint8_t)output->nav.status;
    snapshot->safety_level = (uint8_t)output->safety.level;
    snapshot->terminal_stage = (uint8_t)output->terminal_stage;
    snapshot->recovery_stage = (uint8_t)output->recovery.stage;
    snapshot->target_status = (uint8_t)output->target.status;
    snapshot->docking_stage = (uint8_t)output->mission.docking_stage;
    if (output->armed) snapshot->flags |= NAV_TELEMETRY_FLAG_ARMED;
    if (output->step_valid) snapshot->flags |= NAV_TELEMETRY_FLAG_STEP_VALID;
    if (output->trajectory_valid)
        snapshot->flags |= NAV_TELEMETRY_FLAG_TRAJECTORY_VALID;
    if (output->target.visible)
        snapshot->flags |= NAV_TELEMETRY_FLAG_TARGET_VISIBLE;
    if (output->home.visible)
        snapshot->flags |= NAV_TELEMETRY_FLAG_HOME_VISIBLE;
    if (output->health.watchdog_tripped)
        snapshot->flags |= NAV_TELEMETRY_FLAG_WATCHDOG_TRIPPED;
    if (output->recovery_active)
        snapshot->flags |= NAV_TELEMETRY_FLAG_RECOVERY_ACTIVE;
    if (output->mission.mission_complete)
        snapshot->flags |= NAV_TELEMETRY_FLAG_MISSION_COMPLETE;
    if (output->mission.mission_failed)
        snapshot->flags |= NAV_TELEMETRY_FLAG_MISSION_FAILED;
    return snapshot_valid(snapshot);
}

NavTelemetryStatus nav_telemetry_encode(
    const NavTelemetrySnapshot *snapshot,
    uint8_t frame[NAV_TELEMETRY_FRAME_SIZE])
{
    uint32_t crc;
    if (snapshot == 0 || frame == 0) return NAV_TELEMETRY_ERROR_ARGUMENT;
    if (!snapshot_valid(snapshot)) return NAV_TELEMETRY_ERROR_VALUE;
    memset(frame, 0, NAV_TELEMETRY_FRAME_SIZE);
    frame[0] = 'R'; frame[1] = 'M'; frame[2] = 'N'; frame[3] = 'T';
    put_u16(&frame[4], NAV_TELEMETRY_SCHEMA_VERSION);
    put_u16(&frame[6], NAV_TELEMETRY_FRAME_SIZE);
    put_u32(&frame[8], snapshot->sequence);
    put_u32(&frame[12], snapshot->timestamp_ms);
    put_u32(&frame[16], snapshot->event_flags);
    put_u32(&frame[20], snapshot->safety_reason_mask);
    put_u32(&frame[24], snapshot->stale_source_mask);
    put_u32(&frame[28], snapshot->duplicate_source_mask);
    put_u32(&frame[32], snapshot->out_of_order_source_mask);
    put_u32(&frame[36], snapshot->nonfinite_source_mask);
    put_u32(&frame[40], snapshot->invalid_source_mask);
    put_u32(&frame[44], snapshot->cycle_gap_ms);
    frame[48] = snapshot->mission_state;
    frame[49] = snapshot->transition_reason;
    frame[50] = snapshot->estimator_status;
    frame[51] = snapshot->safety_level;
    frame[52] = snapshot->terminal_stage;
    frame[53] = snapshot->recovery_stage;
    frame[54] = snapshot->target_status;
    frame[55] = snapshot->docking_stage;
    put_u32(&frame[56], snapshot->flags);
    put_float(&frame[60], snapshot->position.x);
    put_float(&frame[64], snapshot->position.y);
    put_float(&frame[68], snapshot->position.z);
    put_float(&frame[72], snapshot->velocity.x);
    put_float(&frame[76], snapshot->velocity.y);
    put_float(&frame[80], snapshot->velocity.z);
    put_float(&frame[84], snapshot->yaw);
    put_float(&frame[88], snapshot->estimator_quality);
    put_float(&frame[92], snapshot->target_relative_position.x);
    put_float(&frame[96], snapshot->target_relative_position.y);
    put_float(&frame[100], snapshot->target_relative_position.z);
    put_float(&frame[104], snapshot->home_relative_position.x);
    put_float(&frame[108], snapshot->home_relative_position.y);
    put_float(&frame[112], snapshot->home_relative_position.z);
    put_float(&frame[116], snapshot->guidance_velocity.x);
    put_float(&frame[120], snapshot->guidance_velocity.y);
    put_float(&frame[124], snapshot->guidance_velocity.z);
    put_float(&frame[128], snapshot->acceleration_command.x);
    put_float(&frame[132], snapshot->acceleration_command.y);
    put_float(&frame[136], snapshot->acceleration_command.z);
    put_float(&frame[140], snapshot->yaw_rate_command);
    put_float(&frame[144], snapshot->remaining_mission_s);
    crc = nav_telemetry_crc32(frame, TELEMETRY_CRC_OFFSET);
    put_u32(&frame[TELEMETRY_CRC_OFFSET], crc);
    return NAV_TELEMETRY_OK;
}

NavTelemetryStatus nav_telemetry_decode(
    const uint8_t *frame, uint16_t frame_size,
    NavTelemetrySnapshot *snapshot)
{
    uint32_t expected_crc;
    if (frame == 0 || snapshot == 0) return NAV_TELEMETRY_ERROR_ARGUMENT;
    if (frame_size != NAV_TELEMETRY_FRAME_SIZE)
        return NAV_TELEMETRY_ERROR_SIZE;
    if (frame[0] != 'R' || frame[1] != 'M' ||
        frame[2] != 'N' || frame[3] != 'T') {
        return NAV_TELEMETRY_ERROR_MAGIC;
    }
    if (get_u16(&frame[4]) != NAV_TELEMETRY_SCHEMA_VERSION)
        return NAV_TELEMETRY_ERROR_VERSION;
    if (get_u16(&frame[6]) != NAV_TELEMETRY_FRAME_SIZE)
        return NAV_TELEMETRY_ERROR_SIZE;
    expected_crc = nav_telemetry_crc32(frame, TELEMETRY_CRC_OFFSET);
    if (get_u32(&frame[TELEMETRY_CRC_OFFSET]) != expected_crc)
        return NAV_TELEMETRY_ERROR_CRC;

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->sequence = get_u32(&frame[8]);
    snapshot->timestamp_ms = get_u32(&frame[12]);
    snapshot->event_flags = get_u32(&frame[16]);
    snapshot->safety_reason_mask = get_u32(&frame[20]);
    snapshot->stale_source_mask = get_u32(&frame[24]);
    snapshot->duplicate_source_mask = get_u32(&frame[28]);
    snapshot->out_of_order_source_mask = get_u32(&frame[32]);
    snapshot->nonfinite_source_mask = get_u32(&frame[36]);
    snapshot->invalid_source_mask = get_u32(&frame[40]);
    snapshot->cycle_gap_ms = get_u32(&frame[44]);
    snapshot->mission_state = frame[48];
    snapshot->transition_reason = frame[49];
    snapshot->estimator_status = frame[50];
    snapshot->safety_level = frame[51];
    snapshot->terminal_stage = frame[52];
    snapshot->recovery_stage = frame[53];
    snapshot->target_status = frame[54];
    snapshot->docking_stage = frame[55];
    snapshot->flags = get_u32(&frame[56]);
    snapshot->position = vec3(get_float(&frame[60]), get_float(&frame[64]),
                              get_float(&frame[68]));
    snapshot->velocity = vec3(get_float(&frame[72]), get_float(&frame[76]),
                              get_float(&frame[80]));
    snapshot->yaw = get_float(&frame[84]);
    snapshot->estimator_quality = get_float(&frame[88]);
    snapshot->target_relative_position = vec3(get_float(&frame[92]),
        get_float(&frame[96]), get_float(&frame[100]));
    snapshot->home_relative_position = vec3(get_float(&frame[104]),
        get_float(&frame[108]), get_float(&frame[112]));
    snapshot->guidance_velocity = vec3(get_float(&frame[116]),
        get_float(&frame[120]), get_float(&frame[124]));
    snapshot->acceleration_command = vec3(get_float(&frame[128]),
        get_float(&frame[132]), get_float(&frame[136]));
    snapshot->yaw_rate_command = get_float(&frame[140]);
    snapshot->remaining_mission_s = get_float(&frame[144]);
    return snapshot_valid(snapshot) ? NAV_TELEMETRY_OK
                                    : NAV_TELEMETRY_ERROR_VALUE;
}

const char *nav_telemetry_status_name(NavTelemetryStatus status)
{
    switch (status) {
    case NAV_TELEMETRY_OK: return "ok";
    case NAV_TELEMETRY_ERROR_ARGUMENT: return "argument";
    case NAV_TELEMETRY_ERROR_MAGIC: return "magic";
    case NAV_TELEMETRY_ERROR_VERSION: return "version";
    case NAV_TELEMETRY_ERROR_SIZE: return "size";
    case NAV_TELEMETRY_ERROR_CRC: return "crc";
    case NAV_TELEMETRY_ERROR_VALUE: return "value";
    default: return "unknown";
    }
}
