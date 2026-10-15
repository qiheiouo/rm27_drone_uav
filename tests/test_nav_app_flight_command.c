#include <math.h>
#include <stdio.h>

#include "nav_tasks.h"
#include "nav_platform.h"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; \
} } while (0)

static uint32_t fake_time_ms;
static FlightState supplied_state;
static uint8_t state_available = 1u;
static NavMissionCommand supplied_command;
static FcuSetpoint last_setpoint;
static uint32_t fcu_write_count;
static uint8_t last_host_authorization;

static FlightState make_state(uint32_t timestamp_ms)
{
    FlightState state;
    state.position_m = vec3(0.0f, 0.0f, 1.2f);
    state.velocity_mps = vec3_zero();
    state.attitude_body_to_nav = quat_identity();
    state.timestamp_ms = timestamp_ms;
    state.valid_mask = FLIGHT_STATE_REQUIRED_MASK;
    state.frame = FLIGHT_STATE_FRAME_LOCAL_XY_Z_UP;
    state.valid = 1u;
    return state;
}

static void step_app(NavApp *app, const NavAppConfig *cfg,
                     uint32_t timestamp_ms)
{
    fake_time_ms = timestamp_ms;
    supplied_state.timestamp_ms = timestamp_ms;
    nav_app_step(app, cfg->nominal_dt_s);
}

uint32_t nav_time_ms(void) { return fake_time_ms; }

int nav_imu_read(ImuSample *out)
{
    out->accel = vec3(0.0f, 0.0f, NAV_GRAVITY);
    out->gyro = vec3_zero();
    out->timestamp_ms = fake_time_ms;
    return 0;
}

int nav_flight_state_read(FlightState *out)
{
    if (!state_available) return -1;
    *out = supplied_state;
    return 0;
}

int nav_tof_read(float *height_m) { *height_m = 1.2f; return 0; }
int nav_flow_read(FlowFrame *out) { (void)out; return -1; }
int nav_camera_target_read(PixelObs *out) { (void)out; return -1; }
int nav_camera_home_read(PixelObs *out) { (void)out; return -1; }
int nav_local_obstacles_read(DynamicObstacleSet *out)
{
    (void)out;
    return -1;
}
int nav_swarm_frame_read(uint8_t *frame, uint16_t capacity, uint16_t *size)
{
    (void)frame;
    (void)capacity;
    (void)size;
    return -1;
}
int nav_swarm_frame_write(const uint8_t *frame, uint16_t size)
{
    (void)frame;
    (void)size;
    return -1;
}
int nav_mission_command_read(NavMissionCommand *out)
{
    *out = supplied_command;
    return 0;
}
int nav_dock_status_read(NavDockStatus *out) { (void)out; return -1; }

int nav_fcu_setpoint_write(const FcuSetpoint *setpoint)
{
    last_setpoint = *setpoint;
    fcu_write_count++;
    return 0;
}

void nav_fcu_set_motion_authorized(uint8_t armed)
{
    last_host_authorization = armed;
}

void nav_log(const char *msg) { (void)msg; }
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
    MissionState active_state;
    uint32_t warmup;

    nav_runtime_config_default(&cfg);
    cfg.flight_state_source = NAV_FLIGHT_STATE_EXTERNAL;
    CHECK(nav_app_init(&app, &cfg) == NAV_CONFIG_ERROR_NONE);
    supplied_state = make_state(10u);
    supplied_command.start = 0u;
    supplied_command.request_return = 0u;
    supplied_command.request_emergency = 0u;
    supplied_command.motion_authorization_request = 0u;

    /* Put the existing mission in a real active state; no parallel FSM. */
    app.runtime.mission.state = MS_OUTBOUND;

    /* Let the existing trajectory/controller path produce a moving command. */
    for (warmup = 1u; warmup <= 50u; warmup++) {
        step_app(&app, &cfg, warmup * 10u);
        CHECK(last_setpoint.motion_enabled == 0u);
    }

    /* Valid state + fresh explicit authorization produces a final velocity. */
    supplied_command.motion_authorization_request = 1u;
    step_app(&app, &cfg, 510u);
    CHECK(app.runtime.output.step_valid == 1u);
    CHECK(app.runtime.output.armed == 1u);
    CHECK(last_setpoint.motion_enabled == 1u);
    CHECK(last_host_authorization == 1u);
    CHECK(last_setpoint.control_mode == FCU_CONTROL_VELOCITY);
    CHECK(last_setpoint.control_frame ==
          FCU_CONTROL_FRAME_LOCAL_XY_Z_UP);
    CHECK(last_setpoint.yaw_control == FCU_YAW_CONTROL_ANGLE);
    CHECK(last_setpoint.valid_mask ==
          (FCU_SETPOINT_VALID_VELOCITY | FCU_SETPOINT_VALID_YAW));
    CHECK(vec3_norm(last_setpoint.velocity_sp) > 0.01f);
    CHECK(fabsf(last_setpoint.velocity_sp.x -
                app.runtime.output.control.velocity_cmd.x) < 1e-5f);
    CHECK(fcu_setpoint_is_fresh(&last_setpoint, 510u,
          510u + NAV_APP_FCU_COMMAND_VALIDITY_MS));
    CHECK(!fcu_setpoint_is_fresh(&last_setpoint, 510u,
          511u + NAV_APP_FCU_COMMAND_VALIDITY_MS));

    active_state = app.runtime.mission.state;
    app.runtime.target_tracker.initialized = 1u;
    app.runtime.target_tracker.out.visible = 1u;
    app.runtime.target_tracker.out.status = TARGET_TRACK_TRACKING;
    app.runtime.target_tracker.out.rel_pos = vec3(1.0f, 0.0f, 0.0f);
    state_available = 0u;
    step_app(&app, &cfg, 520u);
    CHECK(app.runtime.output.step_valid == 0u);
    CHECK(app.runtime.mission.state == active_state);
    CHECK(app.runtime.output.guidance.vel_sp.x == 0.0f);
    CHECK(app.runtime.output.guidance.vel_sp.y == 0.0f);
    CHECK(app.runtime.output.guidance.vel_sp.z == 0.0f);
    CHECK(app.runtime.output.target.status == TARGET_TRACK_LOST);
    CHECK(last_setpoint.motion_enabled == 0u);
    CHECK(last_setpoint.valid_mask == 0u);
    CHECK(last_host_authorization == 0u);

    /* Restored localization with a continuously high request stays disabled. */
    state_available = 1u;
    step_app(&app, &cfg, 530u);
    CHECK(app.runtime.output.step_valid == 1u);
    CHECK(app.runtime.mission.state == active_state);
    CHECK(vec3_norm(app.runtime.output.control.velocity_cmd) > 0.01f);
    CHECK(last_setpoint.motion_enabled == 0u);
    CHECK(last_host_authorization == 0u);

    /* A low request arms the edge detector; only a later high grants motion. */
    supplied_command.motion_authorization_request = 0u;
    step_app(&app, &cfg, 540u);
    CHECK(last_setpoint.motion_enabled == 0u);
    supplied_command.motion_authorization_request = 1u;
    step_app(&app, &cfg, 550u);
    CHECK(last_setpoint.motion_enabled == 1u);
    CHECK(vec3_norm(last_setpoint.velocity_sp) > 0.01f);

    /* Task-side cancellation revokes motion; restart requires a new edge. */
    supplied_command.motion_authorization_request = 0u;
    step_app(&app, &cfg, 560u);
    CHECK(last_setpoint.motion_enabled == 0u);
    CHECK(app.runtime.mission.state == active_state);
    supplied_command.motion_authorization_request = 1u;
    step_app(&app, &cfg, 570u);
    CHECK(last_setpoint.motion_enabled == 1u);

    /* In a same-cycle race, the state fault wins over the new request. */
    supplied_command.motion_authorization_request = 0u;
    step_app(&app, &cfg, 580u);
    state_available = 0u;
    supplied_command.motion_authorization_request = 1u;
    step_app(&app, &cfg, 590u);
    CHECK(last_setpoint.motion_enabled == 0u);
    state_available = 1u;
    step_app(&app, &cfg, 600u);
    CHECK(last_setpoint.motion_enabled == 0u);
    supplied_command.motion_authorization_request = 0u;
    step_app(&app, &cfg, 610u);
    CHECK(last_setpoint.motion_enabled == 0u);
    supplied_command.motion_authorization_request = 1u;
    step_app(&app, &cfg, 620u);
    CHECK(last_setpoint.motion_enabled == 1u);

    CHECK(fcu_write_count == 62u);
    CHECK(app.fcu_tx_drops == 0u);
    CHECK(app.fcu_command_rejections == 0u);

    if (failures == 0) {
        printf("test_nav_app_flight_command: PASS\n");
        return 0;
    }
    printf("test_nav_app_flight_command: %d FAILURES\n", failures);
    return 1;
}
