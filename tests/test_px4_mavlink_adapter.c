#include <math.h>
#include <stdio.h>
#include <string.h>

#include "px4_mavlink_adapter.h"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; \
} } while (0)

static uint8_t nearf(float a, float b)
{
    return fabsf(a - b) <= 1e-4f ? 1u : 0u;
}

static void send_px4_heartbeat(Px4MavlinkAdapter *adapter,
                               uint32_t received_at_ms,
                               uint8_t armed,
                               uint8_t offboard)
{
    mavlink_message_t message;
    uint8_t base_mode = armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0u;
    uint32_t custom_mode = offboard ? (6u << 16) : (3u << 16);
    (void)mavlink_msg_heartbeat_pack(1u, MAV_COMP_ID_AUTOPILOT1, &message,
        MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, base_mode, custom_mode,
        MAV_STATE_ACTIVE);
    CHECK(px4_mavlink_ingest(adapter, &message, received_at_ms) ==
          PX4_MAVLINK_ACCEPTED);
}

static void send_position(Px4MavlinkAdapter *adapter,
                          uint32_t boot_ms,
                          uint32_t received_at_ms)
{
    mavlink_message_t message;
    (void)mavlink_msg_local_position_ned_pack(1u, MAV_COMP_ID_AUTOPILOT1,
        &message, boot_ms, 1.0f, 2.0f, -3.0f, 4.0f, 5.0f, -6.0f);
    CHECK(px4_mavlink_ingest(adapter, &message, received_at_ms) ==
          PX4_MAVLINK_ACCEPTED);
}

static void send_attitude(Px4MavlinkAdapter *adapter,
                          uint32_t received_at_ms,
                          float yaw_ned)
{
    mavlink_message_t message;
    float half_yaw = yaw_ned * 0.5f;
    const float representation_offset[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    (void)mavlink_msg_attitude_quaternion_pack(
        1u, MAV_COMP_ID_AUTOPILOT1, &message, received_at_ms,
        cosf(half_yaw), 0.0f, 0.0f, sinf(half_yaw),
        0.1f, 0.2f, 0.3f, representation_offset);
    CHECK(px4_mavlink_ingest(adapter, &message, received_at_ms) ==
          PX4_MAVLINK_ACCEPTED);
}

static void send_imu(Px4MavlinkAdapter *adapter, uint32_t received_at_ms)
{
    mavlink_message_t message;
    (void)mavlink_msg_highres_imu_pack(
        1u, MAV_COMP_ID_AUTOPILOT1, &message,
        (uint64_t)received_at_ms * 1000u,
        1.0f, 2.0f, -9.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        1000.0f, 0.0f, 0.0f, 20.0f,
        0xffffu, 0u);
    CHECK(px4_mavlink_ingest(adapter, &message, received_at_ms) ==
          PX4_MAVLINK_ACCEPTED);
}

static void send_estimator_status(Px4MavlinkAdapter *adapter,
                                  uint32_t received_at_ms,
                                  uint16_t flags)
{
    mavlink_message_t message;
    (void)mavlink_msg_estimator_status_pack(
        1u, MAV_COMP_ID_AUTOPILOT1, &message,
        (uint64_t)received_at_ms * 1000u, flags,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.1f);
    CHECK(px4_mavlink_ingest(adapter, &message, received_at_ms) ==
          PX4_MAVLINK_ACCEPTED);
}

int main(void)
{
    Px4MavlinkConfig cfg;
    Px4MavlinkAdapter adapter;
    Px4VehicleFeedback feedback;
    FlightState flight_state;
    ImuSample imu;
    FcuSetpoint setpoint;
    mavlink_message_t message;
    mavlink_message_t parsed;
    mavlink_status_t parser_status;
    mavlink_set_position_target_local_ned_t px4_setpoint;
    mavlink_command_long_t command;
    uint8_t frame[MAVLINK_MAX_PACKET_LEN];
    uint16_t frame_size;
    uint16_t index;
    uint8_t parsed_seen = 0u;
    uint32_t age_ms = 0u;
    GuidanceOutput guidance;
    CtrlOutput control;
    Px4MavlinkConfig invalid_cfg;

    px4_mavlink_config_default(&cfg);
    CHECK(px4_mavlink_config_valid(&cfg));
    invalid_cfg = cfg;
    invalid_cfg.ned_to_nav = (Quatf){0.0f, 0.0f, 0.0f, 0.0f};
    CHECK(!px4_mavlink_config_valid(&invalid_cfg));
    px4_mavlink_init(&adapter, &cfg);
    CHECK(!px4_mavlink_get_flight_state(&adapter, 100u, &flight_state,
                                        &age_ms));

    send_px4_heartbeat(&adapter, 100u, 1u, 1u);
    (void)mavlink_msg_heartbeat_pack(
        2u, MAV_COMP_ID_AUTOPILOT1, &message, MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_PX4, 0u, 3u << 16, MAV_STATE_ACTIVE);
    CHECK(px4_mavlink_ingest(&adapter, &message, 100u) ==
          PX4_MAVLINK_IGNORED);
    CHECK(adapter.feedback.target_system_id == 1u);
    send_position(&adapter, 95u, 101u);
    send_attitude(&adapter, 102u, 0.0f);
    send_imu(&adapter, 103u);

    (void)mavlink_msg_local_position_ned_pack(
        1u, MAV_COMP_ID_AUTOPILOT1, &message, 104u,
        NAN, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    CHECK(px4_mavlink_ingest(&adapter, &message, 104u) ==
          PX4_MAVLINK_REJECTED);
    {
        const float representation_offset[4] = {
            0.0f, 0.0f, 0.0f, 0.0f
        };
        (void)mavlink_msg_attitude_quaternion_pack(
            1u, MAV_COMP_ID_AUTOPILOT1, &message, 104u,
            0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, representation_offset);
        CHECK(px4_mavlink_ingest(&adapter, &message, 104u) ==
              PX4_MAVLINK_REJECTED);
    }

    CHECK(px4_mavlink_get_flight_state(&adapter, 104u, &flight_state,
                                       &age_ms));
    CHECK(age_ms == 3u);
    CHECK(nearf(flight_state.position_m.x, 2.0f));
    CHECK(nearf(flight_state.position_m.y, 1.0f));
    CHECK(nearf(flight_state.position_m.z, 3.0f));
    CHECK(nearf(flight_state.velocity_mps.x, 5.0f));
    CHECK(nearf(flight_state.velocity_mps.y, 4.0f));
    CHECK(nearf(flight_state.velocity_mps.z, 6.0f));
    CHECK(nearf(quat_to_yaw(flight_state.attitude_body_to_nav),
                0.5f * NAV_PI));
    CHECK(px4_mavlink_get_imu(&adapter, 104u, &imu, &age_ms));
    CHECK(nearf(imu.accel.x, 1.0f));
    CHECK(nearf(imu.accel.y, -2.0f));
    CHECK(nearf(imu.accel.z, 9.0f));
    CHECK(nearf(imu.gyro.x, 0.1f));
    CHECK(nearf(imu.gyro.y, -0.2f));
    CHECK(nearf(imu.gyro.z, -0.3f));
    adapter.cfg.heartbeat_timeout_ms = 2u;
    CHECK(!px4_mavlink_get_flight_state(&adapter, 104u, &flight_state,
                                        &age_ms));
    adapter.cfg.heartbeat_timeout_ms = cfg.heartbeat_timeout_ms;

    send_estimator_status(&adapter, 105u, ESTIMATOR_ATTITUDE);
    CHECK(!px4_mavlink_get_flight_state(&adapter, 105u, &flight_state,
                                        &age_ms));
    send_estimator_status(&adapter, 106u,
        ESTIMATOR_ATTITUDE | ESTIMATOR_VELOCITY_HORIZ |
        ESTIMATOR_VELOCITY_VERT | ESTIMATOR_POS_HORIZ_REL |
        ESTIMATOR_POS_VERT_ABS);
    CHECK(px4_mavlink_get_flight_state(&adapter, 106u, &flight_state,
                                       &age_ms));
    CHECK(!px4_mavlink_get_imu(&adapter, 204u, &imu, &age_ms));

    px4_mavlink_get_feedback(&adapter, 104u, &feedback);
    CHECK(feedback.link_active);
    CHECK(feedback.armed);
    CHECK(feedback.offboard_active);
    CHECK(feedback.target_system_id == 1u);
    CHECK(!px4_mavlink_get_flight_state(&adapter, 303u, &flight_state,
                                        &age_ms));

    /* PX4 boot-time rollback invalidates the assembled state until refreshed. */
    send_px4_heartbeat(&adapter, 304u, 0u, 0u);
    send_position(&adapter, 1u, 305u);
    CHECK(adapter.restart_count == 1u);
    CHECK(!px4_mavlink_get_flight_state(&adapter, 305u, &flight_state,
                                        &age_ms));
    send_attitude(&adapter, 306u, 0.5f * NAV_PI);
    CHECK(px4_mavlink_get_flight_state(&adapter, 306u, &flight_state,
                                       &age_ms));
    CHECK(nearf(quat_to_yaw(flight_state.attitude_body_to_nav), 0.0f));

    memset(&guidance, 0, sizeof(guidance));
    memset(&control, 0, sizeof(control));
    control.velocity_cmd = vec3(1.0f, 2.0f, 3.0f);
    guidance.yaw_sp = 0.0f;
    CHECK(fcu_setpoint_from_nav(&guidance, &control,
        FCU_CONTROL_VELOCITY, FCU_YAW_CONTROL_ANGLE,
        FCU_CONTROL_FRAME_LOCAL_XY_Z_UP, 1u, 7u, 400u, 100u,
        &setpoint));
    CHECK(px4_mavlink_make_velocity_setpoint(&adapter, &setpoint, 450u,
        &message) == PX4_SETPOINT_READY);
    mavlink_msg_set_position_target_local_ned_decode(&message, &px4_setpoint);
    CHECK(px4_setpoint.time_boot_ms == 146u);
    CHECK(px4_setpoint.coordinate_frame == MAV_FRAME_LOCAL_NED);
    CHECK(nearf(px4_setpoint.vx, 2.0f));
    CHECK(nearf(px4_setpoint.vy, 1.0f));
    CHECK(nearf(px4_setpoint.vz, -3.0f));
    CHECK(nearf(px4_setpoint.yaw, 0.5f * NAV_PI));
    CHECK((px4_setpoint.type_mask & POSITION_TARGET_TYPEMASK_YAW_IGNORE) == 0u);
    CHECK((px4_setpoint.type_mask &
           POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE) != 0u);

    frame_size = mavlink_msg_to_send_buffer(frame, &message);
    memset(&parser_status, 0, sizeof(parser_status));
    for (index = 0u; index < frame_size; index++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, frame[index], &parsed,
                               &parser_status)) {
            parsed_seen = 1u;
        }
    }
    CHECK(parsed_seen);
    CHECK(parsed.msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED);

    setpoint.yaw_control = FCU_YAW_CONTROL_RATE;
    setpoint.yaw_sp = 0.0f;
    setpoint.yaw_rate_sp = 0.5f;
    setpoint.valid_mask = FCU_SETPOINT_VALID_VELOCITY |
        FCU_SETPOINT_VALID_YAW_RATE;
    CHECK(px4_mavlink_make_velocity_setpoint(&adapter, &setpoint, 450u,
        &message) == PX4_SETPOINT_READY);
    mavlink_msg_set_position_target_local_ned_decode(&message, &px4_setpoint);
    CHECK(nearf(px4_setpoint.yaw_rate, -0.5f));
    CHECK((px4_setpoint.type_mask & POSITION_TARGET_TYPEMASK_YAW_IGNORE) != 0u);

    CHECK(px4_mavlink_make_velocity_setpoint(&adapter, &setpoint, 501u,
        &message) == PX4_SETPOINT_EXPIRED);
    fcu_setpoint_disable(8u, 600u, &setpoint);
    CHECK(px4_mavlink_make_velocity_setpoint(&adapter, &setpoint, 900u,
        &message) == PX4_SETPOINT_DISABLED);

    CHECK(px4_mavlink_make_offboard_request(&adapter, &message));
    mavlink_msg_command_long_decode(&message, &command);
    CHECK(command.command == MAV_CMD_DO_SET_MODE);
    CHECK(nearf(command.param2, 6.0f));
    CHECK(px4_mavlink_make_position_hold_request(&adapter, &message));
    mavlink_msg_command_long_decode(&message, &command);
    CHECK(nearf(command.param2, 3.0f));
    CHECK(px4_mavlink_make_arm_request(&adapter, 1u, &message));
    mavlink_msg_command_long_decode(&message, &command);
    CHECK(command.command == MAV_CMD_COMPONENT_ARM_DISARM);
    CHECK(nearf(command.param1, 1.0f));
    CHECK(px4_mavlink_make_message_interval_request(&adapter,
        MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20000, &message));
    mavlink_msg_command_long_decode(&message, &command);
    CHECK(command.command == MAV_CMD_SET_MESSAGE_INTERVAL);
    CHECK(nearf(command.param1, (float)MAVLINK_MSG_ID_LOCAL_POSITION_NED));

    (void)mavlink_msg_command_ack_pack(
        1u, MAV_COMP_ID_AUTOPILOT1, &message,
        MAV_CMD_DO_SET_MODE, MAV_RESULT_DENIED, 0u, 0, 245u,
        MAV_COMP_ID_ONBOARD_COMPUTER);
    CHECK(px4_mavlink_ingest(&adapter, &message, 700u) ==
          PX4_MAVLINK_ACCEPTED);
    px4_mavlink_get_feedback(&adapter, 700u, &feedback);
    CHECK(feedback.last_command_ack == MAV_CMD_DO_SET_MODE);
    CHECK(feedback.last_command_result == MAV_RESULT_DENIED);

    CHECK(fcu_setpoint_from_nav(&guidance, &control,
        FCU_CONTROL_VELOCITY, FCU_YAW_CONTROL_ANGLE,
        FCU_CONTROL_FRAME_LOCAL_XY_Z_UP, 1u, 9u, 700u, 100u,
        &setpoint));
    setpoint.control_mode = FCU_CONTROL_ACCELERATION;
    CHECK(px4_mavlink_make_velocity_setpoint(&adapter, &setpoint, 700u,
        &message) == PX4_SETPOINT_ERROR);

    if (failures == 0) {
        printf("test_px4_mavlink_adapter: PASS\n");
        return 0;
    }
    printf("test_px4_mavlink_adapter: %d FAILURES\n", failures);
    return 1;
}
