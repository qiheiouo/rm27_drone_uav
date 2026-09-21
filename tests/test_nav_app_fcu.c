#include <stdio.h>

#include "nav_tasks.h"
#include "nav_platform.h"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; \
} } while (0)

static uint32_t fake_time_ms;
static uint8_t imu_available = 1u;
static uint8_t fcu_write_rejected;
static uint8_t last_armed;
static uint32_t fcu_write_count;
static FcuSetpoint last_setpoint;

uint32_t nav_time_ms(void)
{
    return fake_time_ms;
}

int nav_imu_read(ImuSample *out)
{
    if (!imu_available) return -1;
    out->accel = vec3(0.0f, 0.0f, NAV_GRAVITY);
    out->gyro = vec3_zero();
    out->timestamp_ms = fake_time_ms;
    return 0;
}

int nav_tof_read(float *height_m)
{
    *height_m = 0.0f;
    return 0;
}

int nav_flow_read(FlowFrame *out) { (void)out; return -1; }
int nav_camera_target_read(PixelObs *out) { (void)out; return -1; }
int nav_camera_home_read(PixelObs *out) { (void)out; return -1; }
int nav_local_obstacles_read(DynamicObstacleSet *out) { (void)out; return -1; }
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
int nav_mission_command_read(NavMissionCommand *out) { (void)out; return -1; }
int nav_dock_status_read(NavDockStatus *out) { (void)out; return -1; }

int nav_fcu_setpoint_write(const FcuSetpoint *setpoint)
{
    if (fcu_write_rejected) return -1;
    last_setpoint = *setpoint;
    fcu_write_count++;
    return 0;
}

void nav_fcu_set_armed(uint8_t armed)
{
    last_armed = armed;
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

    nav_runtime_config_default(&cfg);
    CHECK(nav_app_init(&app, &cfg) == NAV_CONFIG_ERROR_NONE);
    CHECK(app.fcu_command_sequence == 0u);
    CHECK(app.fcu_tx_drops == 0u);

    fake_time_ms = 10u;
    nav_app_step(&app, cfg.nominal_dt_s);
    CHECK(fcu_write_count == 1u);
    CHECK(last_setpoint.sequence == 0u);
    CHECK(last_setpoint.source_timestamp_ms == 10u);
    CHECK(last_setpoint.validity_ms == NAV_APP_FCU_COMMAND_VALIDITY_MS);
    CHECK(last_setpoint.preferred_control == FCU_CONTROL_AUTO);
    CHECK(last_setpoint.motion_enabled == app.runtime.output.armed);
    CHECK(last_armed == app.runtime.output.armed);

    imu_available = 0u;
    fake_time_ms = 20u;
    nav_app_step(&app, cfg.nominal_dt_s);
    CHECK(fcu_write_count == 2u);
    CHECK(last_setpoint.sequence == 1u);
    CHECK(last_setpoint.motion_enabled == 0u);
    CHECK(last_setpoint.valid_mask == 0u);
    CHECK(last_setpoint.validity_ms == 1u);
    CHECK(last_armed == 0u);

    fcu_write_rejected = 1u;
    fake_time_ms = 30u;
    nav_app_step(&app, cfg.nominal_dt_s);
    CHECK(app.fcu_tx_drops == 1u);
    CHECK(app.fcu_command_sequence == 3u);

    if (failures == 0) {
        printf("test_nav_app_fcu: PASS\n");
        return 0;
    }
    printf("test_nav_app_fcu: %d FAILURES\n", failures);
    return 1;
}
