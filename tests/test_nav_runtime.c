#include <stdio.h>

#include "nav_runtime.h"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; \
} } while (0)

static NavRuntimeInput stationary_input(const ImuSample *imu, float dt)
{
    NavRuntimeInput input;
    input.timestamp_ms = imu->timestamp_ms;
    input.imu = imu;
    input.flow = 0;
    input.odometry = 0;
    input.tof_height = 0.0f;
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

int main(void)
{
    NavRuntimeConfig cfg;
    NavRuntime runtime;
    NavRuntimeInput input;
    ImuSample imu;
    OdomSample odometry;
    SwarmView swarm;
    uint32_t errors;

    nav_runtime_config_default(&cfg);
    CHECK(nav_runtime_config_validate(&cfg) == NAV_CONFIG_ERROR_NONE);
    CHECK(nav_runtime_init(&runtime, &cfg) == NAV_CONFIG_ERROR_NONE);
    CHECK(runtime.initialized == 1u);

    imu.accel = vec3(0.0f, 0.0f, NAV_GRAVITY);
    imu.gyro = vec3_zero();
    imu.timestamp_ms = 10u;
    input = stationary_input(&imu, cfg.nominal_dt_s);
    CHECK(nav_runtime_step(&runtime, &input) == 1u);
    CHECK(runtime.output.step_valid == 1u);
    CHECK(vec3_is_finite(runtime.output.control.accel_cmd));
    CHECK(nav_isfinite(runtime.output.control.yaw_rate_cmd));

    input.imu = 0;
    CHECK(nav_runtime_step(&runtime, &input) == 0u);
    CHECK(runtime.output.armed == 0u);
    CHECK(runtime.output.step_valid == 0u);

    nav_runtime_config_default(&cfg);
    cfg.safety.soft_return_deadline_s = cfg.safety.hard_return_deadline_s;
    errors = nav_runtime_config_validate(&cfg);
    CHECK((errors & NAV_CONFIG_ERROR_SAFETY) != 0u);

    nav_runtime_config_default(&cfg);
    cfg.cam_down.fx = 0.0f;
    errors = nav_runtime_config_validate(&cfg);
    CHECK((errors & NAV_CONFIG_ERROR_CAMERA) != 0u);

    nav_runtime_config_default(&cfg);
    wq_init(&cfg.outbound_route);
    errors = nav_runtime_config_validate(&cfg);
    CHECK((errors & NAV_CONFIG_ERROR_ROUTE) != 0u);
    CHECK(nav_runtime_init(&runtime, &cfg) == errors);
    CHECK(runtime.initialized == 0u);

    CHECK(nav_runtime_config_validate(0) == NAV_CONFIG_ERROR_ARGUMENT);
    CHECK(nav_runtime_config_error_name(NAV_CONFIG_ERROR_SAFETY)[0] == 's');

    /* A structural post-check failure remains visible to the safety monitor. */
    nav_runtime_config_default(&cfg);
    cfg.estimator.mode = EST_MODE_TRUTH;
    cfg.planner.limits.workspace_max.x = 0.5f;
    CHECK(nav_runtime_init(&runtime, &cfg) == NAV_CONFIG_ERROR_NONE);
    runtime.mission.state = MS_OUTBOUND;
    odometry.pos = vec3_zero();
    odometry.vel = vec3_zero();
    odometry.yaw = 0.0f;
    odometry.yaw_rate = 0.0f;
    odometry.att = quat_identity();
    odometry.valid = 1u;
    odometry.timestamp_ms = 20u;
    imu.timestamp_ms = 20u;
    input = stationary_input(&imu, cfg.nominal_dt_s);
    input.odometry = &odometry;
    CHECK(nav_runtime_step(&runtime, &input) == 1u);
    CHECK(runtime.output.trajectory_valid == 0u);
    CHECK((runtime.output.event_flags & NAV_EVENT_TRAJECTORY_INVALID) != 0u);
    CHECK((runtime.output.trajectory.check.flags & TRAJ_CHECK_WORKSPACE) != 0u);

    /* Swarm critical risk feeds the same safety path as local obstacles. */
    nav_runtime_config_default(&cfg);
    cfg.estimator.mode = EST_MODE_TRUTH;
    cfg.safety.collision_critical_timeout_s = 0.02f;
    cfg.swarm_avoidance.mode = SWARM_ENABLED;
    CHECK(nav_runtime_init(&runtime, &cfg) == NAV_CONFIG_ERROR_NONE);
    runtime.mission.state = MS_OUTBOUND;
    swarm_view_init(&swarm, cfg.self_agent_id);
    swarm.other_count = 1u;
    swarm.others[0].agent_id = 1u;
    swarm.others[0].valid = 1u;
    swarm.others[0].quality = 1.0f;
    swarm.others[0].age_s = 0.0f;
    swarm.others[0].pos = vec3_zero();
    swarm.others[0].vel = vec3_zero();
    input.swarm = &swarm;
    input.odometry = &odometry;
    CHECK(nav_runtime_step(&runtime, &input) == 1u);
    imu.timestamp_ms += 10u;
    odometry.timestamp_ms += 10u;
    input.timestamp_ms = imu.timestamp_ms;
    CHECK(nav_runtime_step(&runtime, &input) == 1u);
    CHECK(runtime.output.swarm_collision.risk == SWARM_RISK_CRITICAL);
    CHECK(runtime.output.swarm_avoidance.active == 1u);
    CHECK(runtime.output.swarm_avoidance.yielding == 1u);
    CHECK((runtime.output.safety.reason_mask & SAFETY_REASON_COLLISION) != 0u);
    CHECK(runtime.output.safety.request_emergency == 1u);
    CHECK(runtime.output.mission.state == MS_EMERGENCY_STABILIZE);

    if (failures == 0) {
        printf("test_nav_runtime: PASS\n");
        return 0;
    }
    printf("test_nav_runtime: %d FAILURES\n", failures);
    return 1;
}
