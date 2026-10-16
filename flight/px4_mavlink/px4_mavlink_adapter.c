#include "px4_mavlink_adapter.h"

#include <math.h>
#include <string.h>

#define PX4_CUSTOM_MAIN_MODE_AUTO       4u
#define PX4_CUSTOM_MAIN_MODE_OFFBOARD   6u
#define PX4_CUSTOM_SUB_MODE_AUTO_LOITER 3u

static uint8_t finite_quaternion(Quatf q)
{
    float norm_sq;
    if (!nav_isfinite(q.w) || !nav_isfinite(q.x) ||
        !nav_isfinite(q.y) || !nav_isfinite(q.z)) {
        return 0u;
    }
    norm_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    return (norm_sq > 0.5f && norm_sq < 1.5f) ? 1u : 0u;
}

static uint8_t system_status_allows_state(uint8_t status)
{
    return (status == MAV_STATE_STANDBY || status == MAV_STATE_ACTIVE)
        ? 1u : 0u;
}

static uint8_t system_status_allows_valid_estimator(uint8_t status)
{
    /* PX4 v1.17 reports MAV_STATE_UNINIT while disarmed whenever one of its
     * preflight checks is not passing. A fresh ESTIMATOR_STATUS message is
     * the authoritative source for estimator health in that case. */
    return (status == MAV_STATE_UNINIT ||
            system_status_allows_state(status)) ? 1u : 0u;
}

static uint8_t estimator_flags_allow_state(uint32_t flags)
{
    uint32_t position_flags = ESTIMATOR_POS_HORIZ_REL |
        ESTIMATOR_POS_HORIZ_ABS;
    uint32_t height_flags = ESTIMATOR_POS_VERT_ABS |
        ESTIMATOR_POS_VERT_AGL;
    if ((flags & ESTIMATOR_ATTITUDE) == 0u ||
        (flags & ESTIMATOR_VELOCITY_HORIZ) == 0u ||
        (flags & ESTIMATOR_VELOCITY_VERT) == 0u ||
        (flags & position_flags) == 0u ||
        (flags & height_flags) == 0u) {
        return 0u;
    }
    return 1u;
}

static uint8_t message_from_target(const Px4MavlinkAdapter *adapter,
                                   const mavlink_message_t *message)
{
    return adapter->feedback.px4_detected &&
        message->sysid == adapter->feedback.target_system_id ? 1u : 0u;
}

static Vec3f px4_frd_to_internal_flu(Vec3f value)
{
    return vec3(value.x, -value.y, -value.z);
}

static Quatf px4_attitude_to_navigation(const Px4MavlinkAdapter *adapter,
                                        Quatf body_frd_to_ned)
{
    const Quatf frd_from_flu = {0.0f, 1.0f, 0.0f, 0.0f};
    return quat_normalize(quat_mul(adapter->cfg.ned_to_nav,
        quat_mul(body_frd_to_ned, frd_from_flu)));
}

static void reset_stream_state(Px4MavlinkAdapter *adapter)
{
    adapter->state.valid_mask = 0u;
    adapter->position_seen = 0u;
    adapter->attitude_seen = 0u;
    adapter->imu_seen = 0u;
    adapter->estimator_seen = 0u;
    adapter->estimator_flags = 0u;
}

static uint8_t link_is_active(const Px4MavlinkAdapter *adapter,
                              uint32_t now_ms)
{
    if (!adapter->feedback.px4_detected) return 0u;
    return (uint32_t)(now_ms - adapter->feedback.last_heartbeat_ms) <=
        adapter->cfg.heartbeat_timeout_ms ? 1u : 0u;
}

static uint32_t older_timestamp(uint32_t first, uint32_t second)
{
    return (int32_t)(first - second) <= 0 ? first : second;
}

static uint8_t target_available(const Px4MavlinkAdapter *adapter)
{
    return adapter != 0 && adapter->feedback.px4_detected &&
        adapter->feedback.target_system_id != 0u ? 1u : 0u;
}

static uint8_t make_mode_request(const Px4MavlinkAdapter *adapter,
                                 uint8_t main_mode,
                                 uint8_t sub_mode,
                                 mavlink_message_t *message)
{
    if (!target_available(adapter) || message == 0) return 0u;
    (void)mavlink_msg_command_long_pack(
        adapter->cfg.source_system_id,
        adapter->cfg.source_component_id,
        message,
        adapter->feedback.target_system_id,
        adapter->feedback.target_component_id,
        MAV_CMD_DO_SET_MODE,
        0u,
        (float)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        (float)main_mode,
        (float)sub_mode,
        0.0f, 0.0f, 0.0f, 0.0f);
    return 1u;
}

void px4_mavlink_config_default(Px4MavlinkConfig *cfg)
{
    const float inverse_sqrt_two = 0.70710678118f;
    if (cfg == 0) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->source_system_id = 245u;
    cfg->source_component_id = MAV_COMP_ID_ONBOARD_COMPUTER;
    cfg->target_component_id = MAV_COMP_ID_AUTOPILOT1;
    cfg->heartbeat_timeout_ms = 1500u;
    cfg->state_timeout_ms = 200u;
    cfg->imu_timeout_ms = 100u;
    /* SITL fixed alignment: NED north/east/down -> ENU east/north/up. */
    cfg->ned_to_nav.w = 0.0f;
    cfg->ned_to_nav.x = inverse_sqrt_two;
    cfg->ned_to_nav.y = inverse_sqrt_two;
    cfg->ned_to_nav.z = 0.0f;
}

uint8_t px4_mavlink_config_valid(const Px4MavlinkConfig *cfg)
{
    if (cfg == 0 || cfg->source_system_id == 0u ||
        cfg->source_component_id == 0u ||
        cfg->target_component_id == 0u ||
        cfg->heartbeat_timeout_ms == 0u || cfg->state_timeout_ms == 0u ||
        cfg->imu_timeout_ms == 0u || !finite_quaternion(cfg->ned_to_nav)) {
        return 0u;
    }
    return 1u;
}

void px4_mavlink_init(Px4MavlinkAdapter *adapter,
                      const Px4MavlinkConfig *cfg)
{
    Px4MavlinkConfig defaults;
    if (adapter == 0) return;
    memset(adapter, 0, sizeof(*adapter));
    if (px4_mavlink_config_valid(cfg)) {
        adapter->cfg = *cfg;
        adapter->cfg.ned_to_nav = quat_normalize(cfg->ned_to_nav);
    } else {
        px4_mavlink_config_default(&defaults);
        adapter->cfg = defaults;
    }
}

Px4MavlinkIngestResult px4_mavlink_ingest(
    Px4MavlinkAdapter *adapter,
    const mavlink_message_t *message,
    uint32_t received_at_ms)
{
    if (adapter == 0 || message == 0) return PX4_MAVLINK_REJECTED;

    if (message->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        mavlink_heartbeat_t heartbeat;
        mavlink_msg_heartbeat_decode(message, &heartbeat);
        if (heartbeat.autopilot != MAV_AUTOPILOT_PX4) {
            return PX4_MAVLINK_IGNORED;
        }
        if (message->compid != adapter->cfg.target_component_id ||
            (adapter->cfg.target_system_id != 0u &&
             message->sysid != adapter->cfg.target_system_id) ||
            (adapter->feedback.px4_detected &&
             message->sysid != adapter->feedback.target_system_id)) {
            return PX4_MAVLINK_IGNORED;
        }
        adapter->feedback.target_system_id = message->sysid;
        adapter->feedback.target_component_id = message->compid;
        adapter->feedback.px4_detected = 1u;
        adapter->feedback.last_heartbeat_ms = received_at_ms;
        adapter->feedback.last_message_ms = received_at_ms;
        adapter->feedback.base_mode = heartbeat.base_mode;
        adapter->feedback.custom_mode = heartbeat.custom_mode;
        adapter->feedback.system_status = heartbeat.system_status;
        adapter->feedback.armed =
            (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0u;
        adapter->feedback.offboard_active =
            ((heartbeat.custom_mode >> 16) & 0xffu) ==
                PX4_CUSTOM_MAIN_MODE_OFFBOARD ? 1u : 0u;
        return PX4_MAVLINK_ACCEPTED;
    }

    if (!message_from_target(adapter, message)) return PX4_MAVLINK_IGNORED;
    adapter->feedback.last_message_ms = received_at_ms;

    switch (message->msgid) {
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
        mavlink_local_position_ned_t position;
        Vec3f ned_position;
        Vec3f ned_velocity;
        mavlink_msg_local_position_ned_decode(message, &position);
        ned_position = vec3(position.x, position.y, position.z);
        ned_velocity = vec3(position.vx, position.vy, position.vz);
        if (!vec3_is_finite(ned_position) ||
            !vec3_is_finite(ned_velocity)) {
            return PX4_MAVLINK_REJECTED;
        }
        if (adapter->position_seen &&
            (int32_t)(position.time_boot_ms -
                      adapter->last_position_boot_ms) < 0) {
            reset_stream_state(adapter);
            adapter->restart_count++;
        }
        adapter->last_position_boot_ms = position.time_boot_ms;
        adapter->state.source_timestamp_ms = position.time_boot_ms;
        adapter->state.local_position = quat_rotate(
            adapter->cfg.ned_to_nav, ned_position);
        adapter->state.local_velocity = quat_rotate(
            adapter->cfg.ned_to_nav, ned_velocity);
        /* Local Z is relative to the EKF origin and may be negative even on
         * the ground. It is not a non-negative AGL height measurement. */
        adapter->state.valid_mask |= FCU_STATE_VALID_LOCAL_POSITION |
            FCU_STATE_VALID_LOCAL_VELOCITY;
        adapter->position_received_ms = received_at_ms;
        adapter->position_seen = 1u;
        return PX4_MAVLINK_ACCEPTED;
    }
    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: {
        mavlink_attitude_quaternion_t attitude;
        Quatf px4_attitude;
        Vec3f angular_velocity;
        mavlink_msg_attitude_quaternion_decode(message, &attitude);
        px4_attitude.w = attitude.q1;
        px4_attitude.x = attitude.q2;
        px4_attitude.y = attitude.q3;
        px4_attitude.z = attitude.q4;
        angular_velocity = vec3(attitude.rollspeed, attitude.pitchspeed,
                                attitude.yawspeed);
        if (!finite_quaternion(px4_attitude) ||
            !vec3_is_finite(angular_velocity)) {
            return PX4_MAVLINK_REJECTED;
        }
        adapter->state.attitude = px4_attitude_to_navigation(
            adapter, quat_normalize(px4_attitude));
        adapter->state.angular_velocity =
            px4_frd_to_internal_flu(angular_velocity);
        adapter->state.valid_mask |= FCU_STATE_VALID_ATTITUDE |
            FCU_STATE_VALID_ANGULAR_VELOCITY;
        adapter->attitude_received_ms = received_at_ms;
        adapter->attitude_seen = 1u;
        return PX4_MAVLINK_ACCEPTED;
    }
    case MAVLINK_MSG_ID_HIGHRES_IMU: {
        mavlink_highres_imu_t imu;
        Vec3f acceleration;
        const uint16_t required_updates = HIGHRES_IMU_UPDATED_XACC |
            HIGHRES_IMU_UPDATED_YACC | HIGHRES_IMU_UPDATED_ZACC;
        mavlink_msg_highres_imu_decode(message, &imu);
        acceleration = vec3(imu.xacc, imu.yacc, imu.zacc);
        if ((imu.fields_updated & required_updates) != required_updates ||
            !vec3_is_finite(acceleration)) {
            return PX4_MAVLINK_REJECTED;
        }
        adapter->imu_accel_flu = px4_frd_to_internal_flu(acceleration);
        adapter->state.linear_acceleration = adapter->imu_accel_flu;
        adapter->state.valid_mask |= FCU_STATE_VALID_LINEAR_ACCELERATION;
        adapter->imu_received_ms = received_at_ms;
        adapter->imu_seen = 1u;
        return PX4_MAVLINK_ACCEPTED;
    }
    case MAVLINK_MSG_ID_ESTIMATOR_STATUS: {
        mavlink_estimator_status_t estimator;
        mavlink_msg_estimator_status_decode(message, &estimator);
        adapter->estimator_flags = estimator.flags;
        adapter->estimator_received_ms = received_at_ms;
        adapter->estimator_seen = 1u;
        return PX4_MAVLINK_ACCEPTED;
    }
    case MAVLINK_MSG_ID_EXTENDED_SYS_STATE: {
        mavlink_extended_sys_state_t state;
        mavlink_msg_extended_sys_state_decode(message, &state);
        adapter->feedback.landed_state = state.landed_state;
        return PX4_MAVLINK_ACCEPTED;
    }
    case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(message, &ack);
        adapter->feedback.last_command_ack = ack.command;
        adapter->feedback.last_command_result = ack.result;
        adapter->feedback.last_command_ack_ms = received_at_ms;
        return PX4_MAVLINK_ACCEPTED;
    }
    default:
        return PX4_MAVLINK_IGNORED;
    }
}

uint8_t px4_mavlink_get_flight_state(Px4MavlinkAdapter *adapter,
                                     uint32_t now_ms,
                                     FlightState *flight_state,
                                     uint32_t *age_ms)
{
    uint32_t sample_received_ms;
    uint8_t estimator_valid = 1u;
    if (adapter == 0 || flight_state == 0 ||
        !link_is_active(adapter, now_ms) || !adapter->position_seen ||
        !adapter->attitude_seen) {
        return 0u;
    }
    sample_received_ms = older_timestamp(adapter->position_received_ms,
                                         adapter->attitude_received_ms);
    if ((uint32_t)(now_ms - sample_received_ms) >
        adapter->cfg.state_timeout_ms) {
        return 0u;
    }
    if (adapter->estimator_seen) {
        if ((uint32_t)(now_ms - adapter->estimator_received_ms) >
            adapter->cfg.state_timeout_ms) {
            return 0u;
        }
        estimator_valid = estimator_flags_allow_state(
            adapter->estimator_flags) &&
            system_status_allows_valid_estimator(
                adapter->feedback.system_status);
    } else {
        estimator_valid = system_status_allows_state(
            adapter->feedback.system_status);
    }
    adapter->state.sequence = adapter->state_sequence++;
    adapter->state.capabilities = FCU_CAP_VELOCITY_SETPOINT |
        FCU_CAP_YAW_SETPOINT | FCU_CAP_YAW_RATE_SETPOINT |
        FCU_CAP_EXTERNAL_CONTROL;
    adapter->state.armed = adapter->feedback.armed;
    adapter->state.external_control_active =
        adapter->feedback.offboard_active;
    adapter->state.estimator_healthy = estimator_valid;
    if (!fcu_state_to_flight_state(&adapter->state, sample_received_ms,
                                   flight_state)) {
        return 0u;
    }
    if (age_ms != 0) *age_ms = now_ms - sample_received_ms;
    return flight_state->valid;
}

uint8_t px4_mavlink_get_imu(const Px4MavlinkAdapter *adapter,
                            uint32_t now_ms,
                            ImuSample *imu,
                            uint32_t *age_ms)
{
    uint32_t sample_received_ms;
    if (adapter == 0 || imu == 0 || !link_is_active(adapter, now_ms) ||
        !adapter->attitude_seen || !adapter->imu_seen) {
        return 0u;
    }
    sample_received_ms = older_timestamp(adapter->attitude_received_ms,
                                         adapter->imu_received_ms);
    if ((uint32_t)(now_ms - sample_received_ms) >
        adapter->cfg.imu_timeout_ms) {
        return 0u;
    }
    imu->accel = adapter->imu_accel_flu;
    imu->gyro = adapter->state.angular_velocity;
    imu->timestamp_ms = sample_received_ms;
    if (age_ms != 0) *age_ms = now_ms - sample_received_ms;
    return 1u;
}

void px4_mavlink_get_feedback(const Px4MavlinkAdapter *adapter,
                              uint32_t now_ms,
                              Px4VehicleFeedback *feedback)
{
    if (adapter == 0 || feedback == 0) return;
    *feedback = adapter->feedback;
    feedback->link_active = link_is_active(adapter, now_ms);
}

Px4SetpointBuildResult px4_mavlink_make_velocity_setpoint(
    const Px4MavlinkAdapter *adapter,
    const FcuSetpoint *setpoint,
    uint32_t now_ms,
    mavlink_message_t *message)
{
    FcuSetpointDisposition disposition;
    Vec3f ned_velocity;
    Vec3f heading_nav;
    Vec3f heading_ned;
    uint32_t px4_boot_ms;
    float yaw = 0.0f;
    float yaw_rate = 0.0f;
    uint16_t type_mask = POSITION_TARGET_TYPEMASK_X_IGNORE |
        POSITION_TARGET_TYPEMASK_Y_IGNORE |
        POSITION_TARGET_TYPEMASK_Z_IGNORE |
        POSITION_TARGET_TYPEMASK_AX_IGNORE |
        POSITION_TARGET_TYPEMASK_AY_IGNORE |
        POSITION_TARGET_TYPEMASK_AZ_IGNORE;

    if (adapter == 0 || setpoint == 0 || message == 0) {
        return PX4_SETPOINT_ERROR;
    }
    disposition = fcu_setpoint_evaluate(
        setpoint, setpoint->source_timestamp_ms, now_ms);
    if (disposition == FCU_SETPOINT_DISABLED) {
        return PX4_SETPOINT_DISABLED;
    }
    if (disposition == FCU_SETPOINT_EXPIRED) {
        return PX4_SETPOINT_EXPIRED;
    }
    if (disposition != FCU_SETPOINT_ACTIVE) return PX4_SETPOINT_ERROR;
    if (!target_available(adapter) || !link_is_active(adapter, now_ms) ||
        !adapter->position_seen) {
        return PX4_SETPOINT_ERROR;
    }
    if (setpoint->control_mode != FCU_CONTROL_VELOCITY ||
        setpoint->control_frame != FCU_CONTROL_FRAME_LOCAL_XY_Z_UP) {
        return PX4_SETPOINT_ERROR;
    }

    ned_velocity = quat_rotate_inv(adapter->cfg.ned_to_nav,
                                   setpoint->velocity_sp);
    if (setpoint->yaw_control == FCU_YAW_CONTROL_ANGLE) {
        heading_nav = vec3(cosf(setpoint->yaw_sp),
                           sinf(setpoint->yaw_sp), 0.0f);
        heading_ned = quat_rotate_inv(adapter->cfg.ned_to_nav, heading_nav);
        yaw = atan2f(heading_ned.y, heading_ned.x);
        type_mask |= POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    } else if (setpoint->yaw_control == FCU_YAW_CONTROL_RATE) {
        Vec3f rate_ned = quat_rotate_inv(adapter->cfg.ned_to_nav,
            vec3(0.0f, 0.0f, setpoint->yaw_rate_sp));
        yaw_rate = rate_ned.z;
        type_mask |= POSITION_TARGET_TYPEMASK_YAW_IGNORE;
    } else {
        return PX4_SETPOINT_ERROR;
    }
    px4_boot_ms = adapter->last_position_boot_ms +
        (uint32_t)(now_ms - adapter->position_received_ms);

    (void)mavlink_msg_set_position_target_local_ned_pack(
        adapter->cfg.source_system_id,
        adapter->cfg.source_component_id,
        message,
        px4_boot_ms,
        adapter->feedback.target_system_id,
        adapter->feedback.target_component_id,
        MAV_FRAME_LOCAL_NED,
        type_mask,
        NAN, NAN, NAN,
        ned_velocity.x, ned_velocity.y, ned_velocity.z,
        NAN, NAN, NAN,
        yaw, yaw_rate);
    return PX4_SETPOINT_READY;
}

uint8_t px4_mavlink_make_heartbeat(const Px4MavlinkAdapter *adapter,
                                   mavlink_message_t *message)
{
    if (adapter == 0 || message == 0) return 0u;
    (void)mavlink_msg_heartbeat_pack(
        adapter->cfg.source_system_id,
        adapter->cfg.source_component_id,
        message,
        MAV_TYPE_ONBOARD_CONTROLLER,
        MAV_AUTOPILOT_INVALID,
        0u,
        0u,
        MAV_STATE_ACTIVE);
    return 1u;
}

uint8_t px4_mavlink_make_message_interval_request(
    const Px4MavlinkAdapter *adapter,
    uint32_t message_id,
    int32_t interval_us,
    mavlink_message_t *message)
{
    if (!target_available(adapter) || message == 0 || interval_us < -1) {
        return 0u;
    }
    (void)mavlink_msg_command_long_pack(
        adapter->cfg.source_system_id,
        adapter->cfg.source_component_id,
        message,
        adapter->feedback.target_system_id,
        adapter->feedback.target_component_id,
        MAV_CMD_SET_MESSAGE_INTERVAL,
        0u,
        (float)message_id,
        (float)interval_us,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    return 1u;
}

uint8_t px4_mavlink_make_offboard_request(const Px4MavlinkAdapter *adapter,
                                          mavlink_message_t *message)
{
    return make_mode_request(
        adapter, PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0u, message);
}

uint8_t px4_mavlink_make_hold_request(
    const Px4MavlinkAdapter *adapter,
    mavlink_message_t *message)
{
    return make_mode_request(adapter, PX4_CUSTOM_MAIN_MODE_AUTO,
                             PX4_CUSTOM_SUB_MODE_AUTO_LOITER, message);
}

uint8_t px4_mavlink_make_arm_request(const Px4MavlinkAdapter *adapter,
                                     uint8_t arm,
                                     mavlink_message_t *message)
{
    if (!target_available(adapter) || message == 0 || arm > 1u) return 0u;
    (void)mavlink_msg_command_long_pack(
        adapter->cfg.source_system_id,
        adapter->cfg.source_component_id,
        message,
        adapter->feedback.target_system_id,
        adapter->feedback.target_component_id,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0u,
        arm ? 1.0f : 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    return 1u;
}
