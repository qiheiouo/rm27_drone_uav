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
static uint32_t state_read_count;
static uint32_t flow_read_count;
static uint32_t fcu_write_count;
static FcuSetpoint last_setpoint;

static FlightState make_state(uint32_t timestamp_ms, float x)
{
    FlightState state;
    state.position_m = vec3(x, 2.0f, 1.0f);
    state.velocity_mps = vec3(0.3f, 0.0f, 0.0f);
    state.attitude_body_to_nav = quat_identity();
    state.timestamp_ms = timestamp_ms;
    state.valid_mask = FLIGHT_STATE_REQUIRED_MASK;
    state.frame = FLIGHT_STATE_FRAME_LOCAL_XY_Z_UP;
    state.valid = 1u;
    return state;
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
    state_read_count++;
    if (!state_available) return -1;
    *out = supplied_state;
    return 0;
}

int nav_tof_read(float *height_m) { *height_m = 1.0f; return 0; }
int nav_flow_read(FlowFrame *out)
{
    (void)out;
    flow_read_count++;
    return -1;
}
int nav_camera_target_read(PixelObs *out) { (void)out; return -1; }
int nav_camera_home_read(PixelObs *out) { (void)out; return -1; }
int nav_local_obstacles_read(DynamicObstacleSet *out) { (void)out; return -1; }
int nav_mission_command_read(NavMissionCommand *out) { (void)out; return -1; }
int nav_dock_status_read(NavDockStatus *out) { (void)out; return -1; }
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

int nav_fcu_setpoint_write(const FcuSetpoint *setpoint)
{
    last_setpoint = *setpoint;
    fcu_write_count++;
    return 0;
}

void nav_fcu_set_motion_authorized(uint8_t armed) { (void)armed; }
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

    nav_runtime_config_default(&cfg);
    cfg.flight_state_source = NAV_FLIGHT_STATE_EXTERNAL;
    CHECK(nav_app_init(&app, &cfg) == NAV_CONFIG_ERROR_NONE);

    fake_time_ms = 10u;
    supplied_state = make_state(fake_time_ms, 1.25f);
    nav_app_step(&app, cfg.nominal_dt_s);
    CHECK(app.runtime.output.step_valid == 1u);
    CHECK(app.runtime.output.nav.mode == EST_MODE_EXTERNAL);
    CHECK(fabsf(app.runtime.output.nav.pos.x - 1.25f) < 1e-5f);
    CHECK(fabsf(app.runtime.output.swarm_self.pos.x - 1.25f) < 1e-5f);
    CHECK(fabsf(app.runtime.output.swarm_self.vel.x - 0.3f) < 1e-5f);
    CHECK(state_read_count == 1u);
    CHECK(flow_read_count == 0u);
    CHECK(fcu_write_count == 1u);

    state_available = 0u;
    fake_time_ms = 20u;
    nav_app_step(&app, cfg.nominal_dt_s);
    CHECK(app.runtime.output.step_valid == 0u);
    CHECK(app.runtime.output.nav.validity == NAV_VALID_INVALID);
    CHECK((app.runtime.output.health.stale_source_mask &
           NAV_INPUT_SOURCE_FLIGHT_STATE) != 0u);
    CHECK(fcu_write_count == 2u);
    CHECK(last_setpoint.motion_enabled == 0u);

    state_available = 1u;
    fake_time_ms = 30u;
    supplied_state = make_state(fake_time_ms, 2.5f);
    nav_app_step(&app, cfg.nominal_dt_s);
    CHECK(app.runtime.output.step_valid == 1u);
    CHECK(app.runtime.output.nav.status == EST_TRACKING);
    CHECK(fabsf(app.runtime.output.nav.pos.x - 2.5f) < 1e-5f);
    CHECK(state_read_count == 3u);
    CHECK(flow_read_count == 0u);

    if (failures == 0) {
        printf("test_nav_app_flight_state: PASS\n");
        return 0;
    }
    printf("test_nav_app_flight_state: %d FAILURES\n", failures);
    return 1;
}
