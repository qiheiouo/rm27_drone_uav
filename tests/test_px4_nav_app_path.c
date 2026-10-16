#include <math.h>
#include <stdio.h>
#include <string.h>

#include "nav_tasks.h"
#include "nav_platform.h"
#include "px4_mavlink_adapter.h"

static int failures;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; \
} } while (0)

static Px4MavlinkAdapter adapter;
static uint32_t fake_time_ms;
static NavMissionCommand mission_command;
static mavlink_message_t last_output_message;
static uint32_t output_message_count;
static uint8_t host_authorized;

static void ingest(mavlink_message_t *message)
{
    CHECK(px4_mavlink_ingest(&adapter, message, fake_time_ms) ==
          PX4_MAVLINK_ACCEPTED);
}

static void publish_px4_state(uint32_t boot_ms, uint16_t estimator_flags)
{
    mavlink_message_t message;
    const float representation_offset[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    (void)mavlink_msg_heartbeat_pack(
        1u, MAV_COMP_ID_AUTOPILOT1, &message, MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_PX4, 0u, 3u << 16, MAV_STATE_ACTIVE);
    ingest(&message);
    (void)mavlink_msg_local_position_ned_pack(
        1u, MAV_COMP_ID_AUTOPILOT1, &message, boot_ms,
        0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f);
    ingest(&message);
    (void)mavlink_msg_attitude_quaternion_pack(
        1u, MAV_COMP_ID_AUTOPILOT1, &message, boot_ms,
        1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        representation_offset);
    ingest(&message);
    (void)mavlink_msg_highres_imu_pack(
        1u, MAV_COMP_ID_AUTOPILOT1, &message,
        (uint64_t)boot_ms * 1000u,
        0.0f, 0.0f, -NAV_GRAVITY,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        1000.0f, 0.0f, 0.0f, 20.0f,
        HIGHRES_IMU_UPDATED_XACC | HIGHRES_IMU_UPDATED_YACC |
        HIGHRES_IMU_UPDATED_ZACC, 0u);
    ingest(&message);
    (void)mavlink_msg_estimator_status_pack(
        1u, MAV_COMP_ID_AUTOPILOT1, &message,
        (uint64_t)boot_ms * 1000u, estimator_flags,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.1f);
    ingest(&message);
}

uint32_t nav_time_ms(void) { return fake_time_ms; }

int nav_imu_read(ImuSample *out)
{
    return px4_mavlink_get_imu(&adapter, fake_time_ms, out, 0) ? 0 : -1;
}

int nav_flight_state_read(FlightState *out)
{
    return px4_mavlink_get_flight_state(
        &adapter, fake_time_ms, out, 0) ? 0 : -1;
}

int nav_tof_read(float *height_m) { (void)height_m; return -1; }
int nav_flow_read(FlowFrame *out) { (void)out; return -1; }
int nav_camera_target_read(PixelObs *out) { (void)out; return -1; }
int nav_camera_home_read(PixelObs *out) { (void)out; return -1; }
int nav_local_obstacles_read(DynamicObstacleSet *out)
{
    (void)out;
    return -1;
}
int nav_mission_command_read(NavMissionCommand *out)
{
    *out = mission_command;
    return 0;
}
int nav_dock_status_read(NavDockStatus *out)
{
    memset(out, 0, sizeof(*out));
    return 0;
}
int nav_swarm_frame_read(uint8_t *frame, uint16_t capacity, uint16_t *size)
{
    (void)frame; (void)capacity; (void)size;
    return -1;
}
int nav_swarm_frame_write(const uint8_t *frame, uint16_t size)
{
    (void)frame; (void)size;
    return -1;
}
int nav_fcu_setpoint_write(const FcuSetpoint *setpoint)
{
    Px4SetpointBuildResult result = px4_mavlink_make_velocity_setpoint(
        &adapter, setpoint, fake_time_ms, &last_output_message);
    if (result == PX4_SETPOINT_DISABLED) return 0;
    if (result != PX4_SETPOINT_READY) return -1;
    output_message_count++;
    return 0;
}
void nav_fcu_set_motion_authorized(uint8_t authorized)
{
    host_authorized = authorized;
}
void nav_log(const char *message) { (void)message; }
int nav_telemetry_write(const uint8_t *frame, uint16_t size)
{
    (void)frame; (void)size;
    return 0;
}

int main(void)
{
    const uint16_t valid_estimator = ESTIMATOR_ATTITUDE |
        ESTIMATOR_VELOCITY_HORIZ | ESTIMATOR_VELOCITY_VERT |
        ESTIMATOR_POS_HORIZ_REL | ESTIMATOR_POS_VERT_ABS;
    Px4MavlinkConfig adapter_cfg;
    NavAppConfig nav_cfg;
    NavApp app;
    mavlink_set_position_target_local_ned_t decoded;
    uint32_t messages_before_fault;
    uint32_t warmup;

    px4_mavlink_config_default(&adapter_cfg);
    px4_mavlink_init(&adapter, &adapter_cfg);
    nav_runtime_config_default(&nav_cfg);
    nav_cfg.flight_state_source = NAV_FLIGHT_STATE_EXTERNAL;
    CHECK(nav_app_init(&app, &nav_cfg) == NAV_CONFIG_ERROR_NONE);
    app.runtime.mission.state = MS_OUTBOUND;
    memset(&mission_command, 0, sizeof(mission_command));

    for (warmup = 0u; warmup < 50u; warmup++) {
        fake_time_ms = 100u + warmup * 10u;
        publish_px4_state(fake_time_ms, valid_estimator);
        nav_app_step(&app, nav_cfg.nominal_dt_s);
    }
    CHECK(app.runtime.output.step_valid);
    CHECK(!host_authorized);
    CHECK(output_message_count == 0u);

    mission_command.motion_authorization_request = 1u;
    mission_command.motion_authorization_sequence = 1u;
    fake_time_ms = 600u;
    publish_px4_state(600u, valid_estimator);
    nav_app_step(&app, nav_cfg.nominal_dt_s);
    CHECK(host_authorized);
    CHECK(output_message_count == 1u);
    CHECK(last_output_message.msgid ==
          MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED);
    mavlink_msg_set_position_target_local_ned_decode(
        &last_output_message, &decoded);
    CHECK(decoded.coordinate_frame == MAV_FRAME_LOCAL_NED);
    CHECK(sqrtf(decoded.vx * decoded.vx + decoded.vy * decoded.vy +
                decoded.vz * decoded.vz) > 0.01f);

    /* Localization health loss withdraws the only motion output path. */
    messages_before_fault = output_message_count;
    fake_time_ms = 610u;
    publish_px4_state(610u, ESTIMATOR_ATTITUDE);
    nav_app_step(&app, nav_cfg.nominal_dt_s);
    CHECK(!app.runtime.output.step_valid);
    CHECK(!host_authorized);
    CHECK(output_message_count == messages_before_fault);

    /* Fresh state alone and a continuously-high old decision cannot resume. */
    fake_time_ms = 620u;
    publish_px4_state(620u, valid_estimator);
    nav_app_step(&app, nav_cfg.nominal_dt_s);
    CHECK(app.runtime.output.step_valid);
    CHECK(!host_authorized);
    CHECK(output_message_count == messages_before_fault);

    /* A new explicit decision restores this non-target waypoint mission. */
    mission_command.motion_authorization_sequence = 2u;
    fake_time_ms = 630u;
    publish_px4_state(630u, valid_estimator);
    nav_app_step(&app, nav_cfg.nominal_dt_s);
    CHECK(host_authorized);
    CHECK(output_message_count == messages_before_fault + 1u);

    if (failures == 0) {
        printf("test_px4_nav_app_path: PASS\n");
        return 0;
    }
    printf("test_px4_nav_app_path: %d FAILURES\n", failures);
    return 1;
}
