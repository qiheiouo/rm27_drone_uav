#include <stdio.h>

#include "nav_runtime.h"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; \
} } while (0)

static NavRuntimeInput make_input(ImuSample *imu, uint32_t timestamp_ms,
                                  float dt)
{
    NavRuntimeInput input;
    input.timestamp_ms = timestamp_ms;
    input.imu = imu;
    input.flow = 0;
    input.odometry = 0;
    input.tof_height = -1.0f;
    input.target_pixel = 0;
    input.home_pixel = 0;
    input.obstacles = 0;
    input.swarm = 0;
    input.start_command = 0u;
    input.request_return = 0u;
    input.request_emergency = 0u;
    input.dock_contact = 0u;
    input.charging_detected = 0u;
    input.wireless_charge_ready = 0u;
    input.dt = dt;
    return input;
}

static uint8_t log_contains(const NavEventLog *log, NavLogEventCode code)
{
    NavEventRecord record;
    uint16_t index;
    for (index = 0u; index < nav_event_log_count(log); index++) {
        if (nav_event_log_get(log, index, &record) && record.code == code) {
            return 1u;
        }
    }
    return 0u;
}

int main(void)
{
    NavEventLog log;
    NavEventRecord record;
    NavRuntimeConfig cfg;
    NavRuntime runtime;
    NavRuntimeInput input;
    ImuSample imu;
    OdomSample odometry;
    PixelObs target;
    uint32_t index;

    nav_event_log_init(&log);
    for (index = 0u; index < 70u; index++) {
        nav_event_log_push(&log, index * 10u, NAV_LOG_RUNTIME_STARTED,
                           index, (float)index);
    }
    CHECK(nav_event_log_count(&log) == NAV_EVENT_LOG_CAPACITY);
    CHECK(log.dropped == 6u);
    CHECK(nav_event_log_get(&log, 0u, &record) == 1u);
    CHECK(record.sequence == 6u);
    CHECK(nav_event_log_get(&log, NAV_EVENT_LOG_CAPACITY - 1u, &record) == 1u);
    CHECK(record.sequence == 69u);

    nav_runtime_config_default(&cfg);
    cfg.health.max_cycle_gap_ms = 0u;
    CHECK((nav_runtime_config_validate(&cfg) & NAV_CONFIG_ERROR_TIMING) != 0u);

    nav_runtime_config_default(&cfg);
    cfg.estimator.mode = EST_MODE_TRUTH;
    CHECK(nav_runtime_init(&runtime, &cfg) == NAV_CONFIG_ERROR_NONE);
    imu.accel = vec3(0.0f, 0.0f, NAV_GRAVITY);
    imu.gyro = vec3_zero();
    imu.timestamp_ms = 1000u;
    input = make_input(&imu, 1000u, 0.01f);
    odometry.pos = vec3_zero();
    odometry.vel = vec3_zero();
    odometry.yaw = 0.0f;
    odometry.yaw_rate = 0.0f;
    odometry.att = quat_identity();
    odometry.valid = 1u;
    odometry.timestamp_ms = 100u;
    input.odometry = &odometry;
    CHECK(nav_runtime_step(&runtime, &input) == 1u);
    CHECK((runtime.output.health.stale_source_mask &
           NAV_INPUT_SOURCE_ODOMETRY) != 0u);
    CHECK((runtime.output.event_flags & NAV_EVENT_INPUT_REJECTED) != 0u);
    CHECK(log_contains(nav_runtime_event_log(&runtime), NAV_LOG_INPUT_STALE));

    imu.timestamp_ms = 1000u;
    input.timestamp_ms = 1010u;
    input.odometry = 0;
    CHECK(nav_runtime_step(&runtime, &input) == 0u);
    CHECK((runtime.output.health.duplicate_source_mask &
           NAV_INPUT_SOURCE_IMU) != 0u);
    CHECK(runtime.output.armed == 0u);
    imu.timestamp_ms = 990u;
    input.timestamp_ms = 1020u;
    CHECK(nav_runtime_step(&runtime, &input) == 0u);
    CHECK((runtime.output.health.out_of_order_source_mask &
           NAV_INPUT_SOURCE_IMU) != 0u);
    CHECK(log_contains(nav_runtime_event_log(&runtime),
                       NAV_LOG_INPUT_OUT_OF_ORDER));

    nav_runtime_config_default(&cfg);
    cfg.estimator.mode = EST_MODE_TRUTH;
    cfg.health.max_cycle_gap_ms = 15u;
    cfg.health.watchdog_trip_after_overruns = 2u;
    CHECK(nav_runtime_init(&runtime, &cfg) == NAV_CONFIG_ERROR_NONE);
    imu.timestamp_ms = 100u;
    input = make_input(&imu, 100u, 0.01f);
    CHECK(nav_runtime_step(&runtime, &input) == 1u);
    imu.timestamp_ms = 130u;
    input.timestamp_ms = 130u;
    CHECK(nav_runtime_step(&runtime, &input) == 1u);
    CHECK((runtime.output.event_flags & NAV_EVENT_WATCHDOG_OVERRUN) != 0u);
    CHECK(runtime.output.health.watchdog_tripped == 0u);
    imu.timestamp_ms = 160u;
    input.timestamp_ms = 160u;
    CHECK(nav_runtime_step(&runtime, &input) == 1u);
    CHECK(runtime.output.health.watchdog_tripped == 1u);
    CHECK((runtime.output.event_flags & NAV_EVENT_WATCHDOG_TRIPPED) != 0u);
    CHECK(runtime.output.mission.state == MS_EMERGENCY_STABILIZE);
    CHECK(log_contains(nav_runtime_event_log(&runtime), NAV_LOG_WATCHDOG_TRIPPED));

    nav_runtime_config_default(&cfg);
    CHECK(nav_runtime_init(&runtime, &cfg) == NAV_CONFIG_ERROR_NONE);
    imu.timestamp_ms = UINT32_MAX - 5u;
    input = make_input(&imu, UINT32_MAX - 5u, 0.01f);
    CHECK(nav_runtime_step(&runtime, &input) == 1u);
    imu.timestamp_ms = 4u;
    input.timestamp_ms = 4u;
    CHECK(nav_runtime_step(&runtime, &input) == 1u);
    CHECK(runtime.output.health.cycle_gap_ms == 10u);
    CHECK(runtime.output.health.watchdog_tripped == 0u);
    CHECK(runtime.output.health.out_of_order_source_mask == 0u);

    nav_runtime_config_default(&cfg);
    CHECK(nav_runtime_init(&runtime, &cfg) == NAV_CONFIG_ERROR_NONE);
    imu.timestamp_ms = 10u;
    input = make_input(&imu, 10u, 0.01f);
    target.visible = 1u;
    target.u = 160.0f;
    target.v = 120.0f;
    target.size_px = 0.0f;
    target.timestamp_ms = 10u;
    input.target_pixel = &target;
    CHECK(nav_runtime_step(&runtime, &input) == 1u);
    CHECK((runtime.output.health.invalid_source_mask &
           NAV_INPUT_SOURCE_TARGET) != 0u);
    CHECK(runtime.output.target.visible == 0u);

    if (failures == 0) {
        printf("test_diagnostics: PASS\n");
        return 0;
    }
    printf("test_diagnostics: %d FAILURES\n", failures);
    return 1;
}
