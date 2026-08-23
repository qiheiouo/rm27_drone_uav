#include "fcu_bridge.h"

#include <string.h>

#define FCU_LINK_HEADER_SIZE 16u
#define FCU_LINK_STATE_CRC_OFFSET 96u
#define FCU_LINK_SETPOINT_CRC_OFFSET 68u
#define FCU_LINK_MAX_VALIDITY_MS 2000u

static const uint8_t fcu_magic[4] = {'R', 'M', 'F', 'C'};

static void put_u16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xffu);
    dst[1] = (uint8_t)((value >> 8) & 0xffu);
}

static uint16_t get_u16(const uint8_t *src)
{
    return (uint16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8));
}

static void put_u32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xffu);
    dst[1] = (uint8_t)((value >> 8) & 0xffu);
    dst[2] = (uint8_t)((value >> 16) & 0xffu);
    dst[3] = (uint8_t)((value >> 24) & 0xffu);
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
    uint32_t bits = 0u;
    memcpy(&bits, &value, sizeof(bits));
    put_u32(dst, bits);
}

static float get_float(const uint8_t *src)
{
    uint32_t bits = get_u32(src);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void put_vec3(uint8_t *dst, Vec3f value)
{
    put_float(&dst[0], value.x);
    put_float(&dst[4], value.y);
    put_float(&dst[8], value.z);
}

static Vec3f get_vec3(const uint8_t *src)
{
    return vec3(get_float(&src[0]), get_float(&src[4]), get_float(&src[8]));
}

static uint8_t quat_is_valid(Quatf q)
{
    float norm_sq;
    if (!nav_isfinite(q.w) || !nav_isfinite(q.x) ||
        !nav_isfinite(q.y) || !nav_isfinite(q.z)) {
        return 0u;
    }
    norm_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    return (norm_sq >= 0.25f && norm_sq <= 2.25f) ? 1u : 0u;
}

static uint8_t state_is_valid(const FcuStateSnapshot *state)
{
    if (state == 0 || sizeof(float) != 4u ||
        (state->valid_mask & ~FCU_STATE_VALID_KNOWN_MASK) != 0u ||
        (state->capabilities & ~FCU_CAP_KNOWN_MASK) != 0u ||
        state->armed > 1u || state->external_control_active > 1u ||
        state->estimator_healthy > 1u) {
        return 0u;
    }
    if ((state->valid_mask & FCU_STATE_VALID_ATTITUDE) != 0u &&
        !quat_is_valid(state->attitude)) {
        return 0u;
    }
    if ((state->valid_mask & FCU_STATE_VALID_ANGULAR_VELOCITY) != 0u &&
        !vec3_is_finite(state->angular_velocity)) {
        return 0u;
    }
    if ((state->valid_mask & FCU_STATE_VALID_LINEAR_ACCELERATION) != 0u &&
        !vec3_is_finite(state->linear_acceleration)) {
        return 0u;
    }
    if ((state->valid_mask & FCU_STATE_VALID_LOCAL_POSITION) != 0u &&
        !vec3_is_finite(state->local_position)) {
        return 0u;
    }
    if ((state->valid_mask & FCU_STATE_VALID_LOCAL_VELOCITY) != 0u &&
        !vec3_is_finite(state->local_velocity)) {
        return 0u;
    }
    if ((state->valid_mask & FCU_STATE_VALID_HEIGHT) != 0u &&
        (!nav_isfinite(state->height_m) || state->height_m < 0.0f)) {
        return 0u;
    }
    if ((state->valid_mask & FCU_STATE_VALID_BATTERY) != 0u &&
        (!nav_isfinite(state->battery_remaining) ||
         state->battery_remaining < 0.0f || state->battery_remaining > 1.0f)) {
        return 0u;
    }
    return 1u;
}

static uint8_t preference_valid(FcuControlPreference preference)
{
    return (preference == FCU_CONTROL_AUTO ||
            preference == FCU_CONTROL_VELOCITY ||
            preference == FCU_CONTROL_ACCELERATION) ? 1u : 0u;
}

static uint8_t setpoint_is_valid(const FcuSetpoint *setpoint)
{
    if (setpoint == 0 || sizeof(float) != 4u ||
        !preference_valid(setpoint->preferred_control) ||
        setpoint->motion_enabled > 1u || setpoint->validity_ms == 0u ||
        setpoint->validity_ms > FCU_LINK_MAX_VALIDITY_MS ||
        (setpoint->valid_mask & ~FCU_SETPOINT_VALID_KNOWN_MASK) != 0u) {
        return 0u;
    }
    if ((setpoint->valid_mask & FCU_SETPOINT_VALID_POSITION) != 0u &&
        !vec3_is_finite(setpoint->position_sp)) {
        return 0u;
    }
    if ((setpoint->valid_mask & FCU_SETPOINT_VALID_VELOCITY) != 0u &&
        !vec3_is_finite(setpoint->velocity_sp)) {
        return 0u;
    }
    if ((setpoint->valid_mask & FCU_SETPOINT_VALID_ACCELERATION) != 0u &&
        !vec3_is_finite(setpoint->acceleration_sp)) {
        return 0u;
    }
    if ((setpoint->valid_mask & FCU_SETPOINT_VALID_YAW) != 0u &&
        !nav_isfinite(setpoint->yaw_sp)) {
        return 0u;
    }
    if ((setpoint->valid_mask & FCU_SETPOINT_VALID_YAW_RATE) != 0u &&
        !nav_isfinite(setpoint->yaw_rate_sp)) {
        return 0u;
    }
    if (setpoint->motion_enabled) {
        if (setpoint->preferred_control == FCU_CONTROL_VELOCITY &&
            (setpoint->valid_mask & FCU_SETPOINT_VALID_VELOCITY) == 0u) {
            return 0u;
        }
        if (setpoint->preferred_control == FCU_CONTROL_ACCELERATION &&
            (setpoint->valid_mask & FCU_SETPOINT_VALID_ACCELERATION) == 0u) {
            return 0u;
        }
        if (setpoint->preferred_control == FCU_CONTROL_AUTO &&
            (setpoint->valid_mask & (FCU_SETPOINT_VALID_VELOCITY |
                                     FCU_SETPOINT_VALID_ACCELERATION)) == 0u) {
            return 0u;
        }
    }
    return 1u;
}

static uint8_t sequence_newer(uint16_t candidate, uint16_t previous)
{
    uint16_t delta = (uint16_t)(candidate - previous);
    return (delta != 0u && delta < 0x8000u) ? 1u : 0u;
}

static uint16_t state_flags(const FcuStateSnapshot *state)
{
    uint16_t flags = 0u;
    if (state->armed) flags |= FCU_STATE_FLAG_ARMED;
    if (state->external_control_active)
        flags |= FCU_STATE_FLAG_EXTERNAL_CONTROL_ACTIVE;
    if (state->estimator_healthy) flags |= FCU_STATE_FLAG_ESTIMATOR_HEALTHY;
    return flags;
}

static void encode_header(uint8_t *frame, uint8_t type, uint16_t size,
                          uint16_t sequence, uint16_t flags,
                          uint32_t timestamp_ms)
{
    memcpy(frame, fcu_magic, sizeof(fcu_magic));
    frame[4] = FCU_LINK_SCHEMA_VERSION;
    frame[5] = type;
    put_u16(&frame[6], size);
    put_u16(&frame[8], sequence);
    put_u16(&frame[10], flags);
    put_u32(&frame[12], timestamp_ms);
}

static FcuLinkResult validate_header(const uint8_t *frame, uint16_t size,
                                     uint8_t expected_type,
                                     uint16_t expected_size,
                                     uint16_t crc_offset)
{
    if (frame == 0) return FCU_LINK_ERROR_ARGUMENT;
    if (size != expected_size || get_u16(&frame[6]) != expected_size)
        return FCU_LINK_ERROR_SIZE;
    if (memcmp(frame, fcu_magic, sizeof(fcu_magic)) != 0)
        return FCU_LINK_ERROR_MAGIC;
    if (frame[4] != FCU_LINK_SCHEMA_VERSION)
        return FCU_LINK_ERROR_VERSION;
    if (frame[5] != expected_type) return FCU_LINK_ERROR_TYPE;
    if (get_u32(&frame[crc_offset]) != fcu_link_crc32(frame, crc_offset))
        return FCU_LINK_ERROR_CRC;
    return FCU_LINK_OK;
}

void fcu_bridge_config_default(FcuBridgeConfig *cfg)
{
    if (cfg == 0) return;
    cfg->state_timeout_ms = 50u;
    cfg->command_validity_ms = 100u;
    cfg->required_state_mask = FCU_STATE_VALID_ATTITUDE |
        FCU_STATE_VALID_ANGULAR_VELOCITY |
        FCU_STATE_VALID_LINEAR_ACCELERATION |
        FCU_STATE_VALID_LOCAL_POSITION |
        FCU_STATE_VALID_LOCAL_VELOCITY;
    cfg->preferred_control = FCU_CONTROL_AUTO;
}

uint8_t fcu_bridge_config_valid(const FcuBridgeConfig *cfg)
{
    if (cfg == 0 || cfg->state_timeout_ms == 0u ||
        cfg->state_timeout_ms > 2000u || cfg->command_validity_ms == 0u ||
        cfg->command_validity_ms > FCU_LINK_MAX_VALIDITY_MS ||
        (cfg->required_state_mask & ~FCU_STATE_VALID_KNOWN_MASK) != 0u ||
        !preference_valid(cfg->preferred_control)) {
        return 0u;
    }
    return 1u;
}

void fcu_bridge_init(FcuBridge *bridge, const FcuBridgeConfig *cfg)
{
    FcuBridgeConfig defaults;
    if (bridge == 0) return;
    memset(bridge, 0, sizeof(*bridge));
    if (fcu_bridge_config_valid(cfg)) {
        bridge->cfg = *cfg;
    } else {
        fcu_bridge_config_default(&defaults);
        bridge->cfg = defaults;
    }
    bridge->status = FCU_BRIDGE_WAITING;
}

FcuBridgeStatus fcu_bridge_update(FcuBridge *bridge, uint32_t now_ms)
{
    uint32_t age_ms;
    if (bridge == 0 || !bridge->state_seen) return FCU_BRIDGE_WAITING;
    age_ms = now_ms - bridge->received_at_ms;
    if (age_ms <= bridge->cfg.state_timeout_ms) {
        bridge->status = FCU_BRIDGE_ACTIVE;
        bridge->stale_latched = 0u;
    } else {
        bridge->status = FCU_BRIDGE_STALE;
        if (!bridge->stale_latched) {
            bridge->stats.stale_transitions++;
            bridge->stale_latched = 1u;
        }
    }
    return bridge->status;
}

FcuBridgeIngestResult fcu_bridge_ingest_state(FcuBridge *bridge,
                                               const FcuStateSnapshot *state,
                                               uint32_t received_at_ms)
{
    uint8_t was_stale;
    if (bridge == 0 || !state_is_valid(state) ||
        (state->valid_mask & bridge->cfg.required_state_mask) !=
            bridge->cfg.required_state_mask) {
        if (bridge != 0) bridge->stats.invalid_states++;
        return FCU_BRIDGE_REJECTED_INVALID;
    }

    was_stale = (fcu_bridge_update(bridge, received_at_ms) == FCU_BRIDGE_STALE)
        ? 1u : 0u;
    if (bridge->state_seen && !was_stale) {
        if (state->sequence == bridge->latest_state.sequence) {
            bridge->stats.duplicate_states++;
            return FCU_BRIDGE_REJECTED_DUPLICATE;
        }
        if (!sequence_newer(state->sequence, bridge->latest_state.sequence)) {
            bridge->stats.out_of_order_states++;
            return FCU_BRIDGE_REJECTED_OUT_OF_ORDER;
        }
    }

    bridge->latest_state = *state;
    bridge->latest_state.attitude = quat_normalize(state->attitude);
    bridge->received_at_ms = received_at_ms;
    bridge->state_seen = 1u;
    bridge->status = FCU_BRIDGE_ACTIVE;
    bridge->stale_latched = 0u;
    if (was_stale) {
        bridge->stats.resynchronized_states++;
        return FCU_BRIDGE_RESYNCHRONIZED;
    }
    bridge->stats.accepted_states++;
    return FCU_BRIDGE_ACCEPTED;
}

uint8_t fcu_bridge_get_state(FcuBridge *bridge, uint32_t now_ms,
                             FcuStateSnapshot *state, uint32_t *age_ms)
{
    if (bridge == 0 || state == 0 ||
        fcu_bridge_update(bridge, now_ms) != FCU_BRIDGE_ACTIVE) {
        return 0u;
    }
    *state = bridge->latest_state;
    if (age_ms != 0) *age_ms = now_ms - bridge->received_at_ms;
    return 1u;
}

uint8_t fcu_setpoint_from_nav(const GuidanceOutput *guidance,
                              const CtrlOutput *control,
                              FcuControlPreference preferred_control,
                              uint8_t motion_enabled,
                              uint16_t sequence,
                              uint32_t timestamp_ms,
                              uint16_t validity_ms,
                              FcuSetpoint *setpoint)
{
    if (guidance == 0 || control == 0 || setpoint == 0 ||
        !preference_valid(preferred_control) || motion_enabled > 1u ||
        validity_ms == 0u || validity_ms > FCU_LINK_MAX_VALIDITY_MS) {
        return 0u;
    }
    memset(setpoint, 0, sizeof(*setpoint));
    setpoint->sequence = sequence;
    setpoint->source_timestamp_ms = timestamp_ms;
    setpoint->validity_ms = validity_ms;
    setpoint->preferred_control = preferred_control;
    setpoint->motion_enabled = motion_enabled;
    if (!motion_enabled) return 1u;

    setpoint->position_sp = guidance->pos_sp;
    setpoint->velocity_sp = guidance->vel_sp;
    setpoint->acceleration_sp = guidance->use_accel_sp
        ? guidance->accel_sp : control->accel_cmd;
    setpoint->yaw_sp = guidance->yaw_sp;
    setpoint->yaw_rate_sp = control->yaw_rate_cmd;
    if (guidance->use_pos_sp)
        setpoint->valid_mask |= FCU_SETPOINT_VALID_POSITION;
    setpoint->valid_mask |= FCU_SETPOINT_VALID_VELOCITY |
        FCU_SETPOINT_VALID_ACCELERATION |
        FCU_SETPOINT_VALID_YAW |
        FCU_SETPOINT_VALID_YAW_RATE;
    return setpoint_is_valid(setpoint);
}

void fcu_setpoint_disable(uint16_t sequence, uint32_t timestamp_ms,
                          FcuSetpoint *setpoint)
{
    if (setpoint == 0) return;
    memset(setpoint, 0, sizeof(*setpoint));
    setpoint->sequence = sequence;
    setpoint->source_timestamp_ms = timestamp_ms;
    setpoint->validity_ms = 1u;
    setpoint->preferred_control = FCU_CONTROL_AUTO;
}

uint8_t fcu_bridge_make_setpoint(FcuBridge *bridge,
                                 const GuidanceOutput *guidance,
                                 const CtrlOutput *control,
                                 uint8_t motion_enabled,
                                 uint32_t now_ms,
                                 FcuSetpoint *setpoint)
{
    FcuControlPreference selected;
    uint32_t capabilities;
    if (bridge == 0 || setpoint == 0 ||
        fcu_bridge_update(bridge, now_ms) != FCU_BRIDGE_ACTIVE) {
        return 0u;
    }
    capabilities = bridge->latest_state.capabilities;
    if ((capabilities & FCU_CAP_EXTERNAL_CONTROL) == 0u) {
        bridge->stats.unsupported_commands++;
        return 0u;
    }

    selected = bridge->cfg.preferred_control;
    if (selected == FCU_CONTROL_AUTO) {
        if ((capabilities & FCU_CAP_VELOCITY_SETPOINT) != 0u) {
            selected = FCU_CONTROL_VELOCITY;
        } else if ((capabilities & FCU_CAP_ACCELERATION_SETPOINT) != 0u) {
            selected = FCU_CONTROL_ACCELERATION;
        } else {
            bridge->stats.unsupported_commands++;
            return 0u;
        }
    }
    if ((selected == FCU_CONTROL_VELOCITY &&
         (capabilities & FCU_CAP_VELOCITY_SETPOINT) == 0u) ||
        (selected == FCU_CONTROL_ACCELERATION &&
         (capabilities & FCU_CAP_ACCELERATION_SETPOINT) == 0u)) {
        bridge->stats.unsupported_commands++;
        return 0u;
    }

    if (!fcu_setpoint_from_nav(guidance, control, selected, motion_enabled,
                               bridge->next_command_sequence, now_ms,
                               bridge->cfg.command_validity_ms, setpoint)) {
        return 0u;
    }
    bridge->next_command_sequence++;
    if (selected == FCU_CONTROL_VELOCITY) {
        setpoint->valid_mask &= ~FCU_SETPOINT_VALID_ACCELERATION;
    } else {
        setpoint->valid_mask &= ~FCU_SETPOINT_VALID_VELOCITY;
    }
    if ((capabilities & FCU_CAP_YAW_SETPOINT) != 0u) {
        setpoint->valid_mask &= ~FCU_SETPOINT_VALID_YAW_RATE;
    } else if ((capabilities & FCU_CAP_YAW_RATE_SETPOINT) != 0u) {
        setpoint->valid_mask &= ~FCU_SETPOINT_VALID_YAW;
    } else {
        setpoint->valid_mask &= ~(FCU_SETPOINT_VALID_YAW |
                                  FCU_SETPOINT_VALID_YAW_RATE);
    }
    if (!motion_enabled) setpoint->valid_mask = 0u;
    return setpoint_is_valid(setpoint);
}

uint8_t fcu_state_to_nav_samples(const FcuStateSnapshot *state,
                                 uint32_t local_timestamp_ms,
                                 ImuSample *imu,
                                 OdomSample *odometry,
                                 float *height_m)
{
    uint32_t required = FCU_STATE_VALID_ATTITUDE |
        FCU_STATE_VALID_ANGULAR_VELOCITY |
        FCU_STATE_VALID_LINEAR_ACCELERATION |
        FCU_STATE_VALID_LOCAL_POSITION |
        FCU_STATE_VALID_LOCAL_VELOCITY;
    if (state == 0 || imu == 0 || odometry == 0 || height_m == 0 ||
        !state_is_valid(state) ||
        (state->valid_mask & required) != required) {
        return 0u;
    }
    imu->accel = state->linear_acceleration;
    imu->gyro = state->angular_velocity;
    imu->timestamp_ms = local_timestamp_ms;
    odometry->pos = state->local_position;
    odometry->vel = state->local_velocity;
    odometry->att = quat_normalize(state->attitude);
    odometry->yaw = quat_to_yaw(odometry->att);
    odometry->yaw_rate = state->angular_velocity.z;
    odometry->timestamp_ms = local_timestamp_ms;
    odometry->valid = state->estimator_healthy;
    *height_m = (state->valid_mask & FCU_STATE_VALID_HEIGHT) != 0u
        ? state->height_m : -1.0f;
    return 1u;
}

uint8_t fcu_setpoint_is_fresh(const FcuSetpoint *setpoint,
                              uint32_t received_at_ms, uint32_t now_ms)
{
    if (!setpoint_is_valid(setpoint) || !setpoint->motion_enabled) return 0u;
    return ((uint32_t)(now_ms - received_at_ms) <= setpoint->validity_ms)
        ? 1u : 0u;
}

uint32_t fcu_link_crc32(const uint8_t *data, uint16_t size)
{
    uint32_t crc = 0xffffffffu;
    uint16_t index;
    uint8_t bit;
    if (data == 0) return 0u;
    for (index = 0u; index < size; index++) {
        crc ^= data[index];
        for (bit = 0u; bit < 8u; bit++) {
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xedb88320u : 0u);
        }
    }
    return ~crc;
}

FcuLinkResult fcu_link_encode_state(
    const FcuStateSnapshot *state,
    uint8_t frame[FCU_LINK_STATE_FRAME_SIZE])
{
    if (frame == 0 || !state_is_valid(state)) return FCU_LINK_ERROR_VALUE;
    memset(frame, 0, FCU_LINK_STATE_FRAME_SIZE);
    encode_header(frame, FCU_LINK_FRAME_TYPE_STATE, FCU_LINK_STATE_FRAME_SIZE,
                  state->sequence, state_flags(state),
                  state->source_timestamp_ms);
    put_u32(&frame[16], state->valid_mask);
    put_u32(&frame[20], state->capabilities);
    if ((state->valid_mask & FCU_STATE_VALID_ATTITUDE) != 0u) {
        put_float(&frame[24], state->attitude.w);
        put_float(&frame[28], state->attitude.x);
        put_float(&frame[32], state->attitude.y);
        put_float(&frame[36], state->attitude.z);
    }
    if ((state->valid_mask & FCU_STATE_VALID_ANGULAR_VELOCITY) != 0u)
        put_vec3(&frame[40], state->angular_velocity);
    if ((state->valid_mask & FCU_STATE_VALID_LINEAR_ACCELERATION) != 0u)
        put_vec3(&frame[52], state->linear_acceleration);
    if ((state->valid_mask & FCU_STATE_VALID_LOCAL_POSITION) != 0u)
        put_vec3(&frame[64], state->local_position);
    if ((state->valid_mask & FCU_STATE_VALID_LOCAL_VELOCITY) != 0u)
        put_vec3(&frame[76], state->local_velocity);
    if ((state->valid_mask & FCU_STATE_VALID_HEIGHT) != 0u)
        put_float(&frame[88], state->height_m);
    if ((state->valid_mask & FCU_STATE_VALID_BATTERY) != 0u)
        put_float(&frame[92], state->battery_remaining);
    put_u32(&frame[FCU_LINK_STATE_CRC_OFFSET],
            fcu_link_crc32(frame, FCU_LINK_STATE_CRC_OFFSET));
    return FCU_LINK_OK;
}

FcuLinkResult fcu_link_decode_state(const uint8_t *frame, uint16_t size,
                                    FcuStateSnapshot *state)
{
    FcuLinkResult result;
    uint16_t flags;
    if (state == 0) return FCU_LINK_ERROR_ARGUMENT;
    result = validate_header(frame, size, FCU_LINK_FRAME_TYPE_STATE,
                             FCU_LINK_STATE_FRAME_SIZE,
                             FCU_LINK_STATE_CRC_OFFSET);
    if (result != FCU_LINK_OK) return result;
    flags = get_u16(&frame[10]);
    if ((flags & ~FCU_STATE_FLAG_KNOWN_MASK) != 0u)
        return FCU_LINK_ERROR_FLAGS;
    memset(state, 0, sizeof(*state));
    state->sequence = get_u16(&frame[8]);
    state->source_timestamp_ms = get_u32(&frame[12]);
    state->valid_mask = get_u32(&frame[16]);
    state->capabilities = get_u32(&frame[20]);
    state->attitude = quat_identity();
    if ((state->valid_mask & FCU_STATE_VALID_ATTITUDE) != 0u) {
        state->attitude.w = get_float(&frame[24]);
        state->attitude.x = get_float(&frame[28]);
        state->attitude.y = get_float(&frame[32]);
        state->attitude.z = get_float(&frame[36]);
    }
    if ((state->valid_mask & FCU_STATE_VALID_ANGULAR_VELOCITY) != 0u)
        state->angular_velocity = get_vec3(&frame[40]);
    if ((state->valid_mask & FCU_STATE_VALID_LINEAR_ACCELERATION) != 0u)
        state->linear_acceleration = get_vec3(&frame[52]);
    if ((state->valid_mask & FCU_STATE_VALID_LOCAL_POSITION) != 0u)
        state->local_position = get_vec3(&frame[64]);
    if ((state->valid_mask & FCU_STATE_VALID_LOCAL_VELOCITY) != 0u)
        state->local_velocity = get_vec3(&frame[76]);
    if ((state->valid_mask & FCU_STATE_VALID_HEIGHT) != 0u)
        state->height_m = get_float(&frame[88]);
    if ((state->valid_mask & FCU_STATE_VALID_BATTERY) != 0u)
        state->battery_remaining = get_float(&frame[92]);
    state->armed = (flags & FCU_STATE_FLAG_ARMED) != 0u ? 1u : 0u;
    state->external_control_active =
        (flags & FCU_STATE_FLAG_EXTERNAL_CONTROL_ACTIVE) != 0u ? 1u : 0u;
    state->estimator_healthy =
        (flags & FCU_STATE_FLAG_ESTIMATOR_HEALTHY) != 0u ? 1u : 0u;
    return state_is_valid(state) ? FCU_LINK_OK : FCU_LINK_ERROR_VALUE;
}

FcuLinkResult fcu_link_encode_setpoint(
    const FcuSetpoint *setpoint,
    uint8_t frame[FCU_LINK_SETPOINT_FRAME_SIZE])
{
    if (frame == 0 || !setpoint_is_valid(setpoint))
        return FCU_LINK_ERROR_VALUE;
    memset(frame, 0, FCU_LINK_SETPOINT_FRAME_SIZE);
    encode_header(frame, FCU_LINK_FRAME_TYPE_SETPOINT,
                  FCU_LINK_SETPOINT_FRAME_SIZE, setpoint->sequence, 0u,
                  setpoint->source_timestamp_ms);
    put_u32(&frame[16], setpoint->valid_mask);
    frame[20] = (uint8_t)setpoint->preferred_control;
    frame[21] = setpoint->motion_enabled;
    put_u16(&frame[22], setpoint->validity_ms);
    if ((setpoint->valid_mask & FCU_SETPOINT_VALID_POSITION) != 0u)
        put_vec3(&frame[24], setpoint->position_sp);
    if ((setpoint->valid_mask & FCU_SETPOINT_VALID_VELOCITY) != 0u)
        put_vec3(&frame[36], setpoint->velocity_sp);
    if ((setpoint->valid_mask & FCU_SETPOINT_VALID_ACCELERATION) != 0u)
        put_vec3(&frame[48], setpoint->acceleration_sp);
    if ((setpoint->valid_mask & FCU_SETPOINT_VALID_YAW) != 0u)
        put_float(&frame[60], setpoint->yaw_sp);
    if ((setpoint->valid_mask & FCU_SETPOINT_VALID_YAW_RATE) != 0u)
        put_float(&frame[64], setpoint->yaw_rate_sp);
    put_u32(&frame[FCU_LINK_SETPOINT_CRC_OFFSET],
            fcu_link_crc32(frame, FCU_LINK_SETPOINT_CRC_OFFSET));
    return FCU_LINK_OK;
}

FcuLinkResult fcu_link_decode_setpoint(const uint8_t *frame, uint16_t size,
                                       FcuSetpoint *setpoint)
{
    FcuLinkResult result;
    if (setpoint == 0) return FCU_LINK_ERROR_ARGUMENT;
    result = validate_header(frame, size, FCU_LINK_FRAME_TYPE_SETPOINT,
                             FCU_LINK_SETPOINT_FRAME_SIZE,
                             FCU_LINK_SETPOINT_CRC_OFFSET);
    if (result != FCU_LINK_OK) return result;
    if (get_u16(&frame[10]) != 0u) return FCU_LINK_ERROR_FLAGS;
    memset(setpoint, 0, sizeof(*setpoint));
    setpoint->sequence = get_u16(&frame[8]);
    setpoint->source_timestamp_ms = get_u32(&frame[12]);
    setpoint->valid_mask = get_u32(&frame[16]);
    setpoint->preferred_control = (FcuControlPreference)frame[20];
    setpoint->motion_enabled = frame[21];
    setpoint->validity_ms = get_u16(&frame[22]);
    if ((setpoint->valid_mask & FCU_SETPOINT_VALID_POSITION) != 0u)
        setpoint->position_sp = get_vec3(&frame[24]);
    if ((setpoint->valid_mask & FCU_SETPOINT_VALID_VELOCITY) != 0u)
        setpoint->velocity_sp = get_vec3(&frame[36]);
    if ((setpoint->valid_mask & FCU_SETPOINT_VALID_ACCELERATION) != 0u)
        setpoint->acceleration_sp = get_vec3(&frame[48]);
    if ((setpoint->valid_mask & FCU_SETPOINT_VALID_YAW) != 0u)
        setpoint->yaw_sp = get_float(&frame[60]);
    if ((setpoint->valid_mask & FCU_SETPOINT_VALID_YAW_RATE) != 0u)
        setpoint->yaw_rate_sp = get_float(&frame[64]);
    return setpoint_is_valid(setpoint) ? FCU_LINK_OK : FCU_LINK_ERROR_VALUE;
}

void fcu_link_parser_init(FcuLinkParser *parser)
{
    if (parser != 0) memset(parser, 0, sizeof(*parser));
}

static void parser_restart(FcuLinkParser *parser, uint8_t byte)
{
    parser->count = 0u;
    parser->expected_size = 0u;
    if (byte == fcu_magic[0]) {
        parser->data[0] = byte;
        parser->count = 1u;
    }
}

FcuLinkParseResult fcu_link_parser_push_byte(FcuLinkParser *parser,
                                             uint8_t byte,
                                             FcuLinkFrame *frame)
{
    uint16_t declared_size;
    uint16_t expected_size;
    if (parser == 0 || frame == 0) return FCU_LINK_PARSE_REJECTED;
    if (parser->count == 0u) {
        if (byte == fcu_magic[0]) {
            parser->data[0] = byte;
            parser->count = 1u;
            return FCU_LINK_PARSE_NONE;
        }
        parser->rejected_headers++;
        return FCU_LINK_PARSE_REJECTED;
    }
    if (parser->count >= FCU_LINK_MAX_FRAME_SIZE) {
        parser->rejected_headers++;
        parser_restart(parser, byte);
        return FCU_LINK_PARSE_REJECTED;
    }
    parser->data[parser->count++] = byte;
    if (parser->count <= sizeof(fcu_magic) &&
        parser->data[parser->count - 1u] != fcu_magic[parser->count - 1u]) {
        parser->rejected_headers++;
        parser_restart(parser, byte);
        return FCU_LINK_PARSE_REJECTED;
    }
    if (parser->count == 8u) {
        declared_size = get_u16(&parser->data[6]);
        expected_size = parser->data[5] == FCU_LINK_FRAME_TYPE_STATE
            ? FCU_LINK_STATE_FRAME_SIZE
            : (parser->data[5] == FCU_LINK_FRAME_TYPE_SETPOINT
                ? FCU_LINK_SETPOINT_FRAME_SIZE : 0u);
        if (parser->data[4] != FCU_LINK_SCHEMA_VERSION ||
            expected_size == 0u || declared_size != expected_size) {
            parser->rejected_headers++;
            parser_restart(parser, byte);
            return FCU_LINK_PARSE_REJECTED;
        }
        parser->expected_size = expected_size;
    }
    if (parser->expected_size != 0u &&
        parser->count == parser->expected_size) {
        memcpy(frame->data, parser->data, parser->expected_size);
        frame->size = parser->expected_size;
        parser->completed_frames++;
        parser->count = 0u;
        parser->expected_size = 0u;
        return FCU_LINK_PARSE_FRAME;
    }
    return FCU_LINK_PARSE_NONE;
}

const char *fcu_bridge_status_name(FcuBridgeStatus status)
{
    switch (status) {
    case FCU_BRIDGE_WAITING: return "WAITING";
    case FCU_BRIDGE_ACTIVE: return "ACTIVE";
    case FCU_BRIDGE_STALE: return "STALE";
    default: return "?";
    }
}

const char *fcu_link_result_name(FcuLinkResult result)
{
    switch (result) {
    case FCU_LINK_OK: return "OK";
    case FCU_LINK_ERROR_ARGUMENT: return "ARGUMENT";
    case FCU_LINK_ERROR_SIZE: return "SIZE";
    case FCU_LINK_ERROR_MAGIC: return "MAGIC";
    case FCU_LINK_ERROR_VERSION: return "VERSION";
    case FCU_LINK_ERROR_TYPE: return "TYPE";
    case FCU_LINK_ERROR_FLAGS: return "FLAGS";
    case FCU_LINK_ERROR_CRC: return "CRC";
    case FCU_LINK_ERROR_VALUE: return "VALUE";
    default: return "?";
    }
}
