#include <math.h>
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
    input.flight_state = 0;
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

static FlightState make_flight_state(uint32_t timestamp_ms)
{
    FlightState state;
    state.position_m = vec3(1.0f, -2.0f, 1.5f);
    state.velocity_mps = vec3(0.4f, -0.2f, 0.1f);
    state.attitude_body_to_nav =
        quat_from_axis_angle(vec3(0.0f, 0.0f, 1.0f), 0.4f);
    state.timestamp_ms = timestamp_ms;
    state.valid_mask = FLIGHT_STATE_REQUIRED_MASK;
    state.frame = FLIGHT_STATE_FRAME_LOCAL_XY_Z_UP;
    state.valid = 1u;
    return state;
}

int main(void)
{
    NavRuntimeConfig cfg;
    NavRuntime runtime;
    NavRuntimeInput input;
    ImuSample imu;
    OdomSample odometry;
    FlightState flight_state;
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
    CHECK(runtime.output.flight_state_source ==
          NAV_FLIGHT_STATE_INTERNAL_ESTIMATOR);
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
    cfg.swarm_link_guard.enabled = 2u;
    cfg.swarm_link_guard.recovery_confirm_s = 0.0f;
    errors = nav_runtime_config_validate(&cfg);
    CHECK((errors & NAV_CONFIG_ERROR_SWARM) != 0u);

    nav_runtime_config_default(&cfg);
    cfg.swarm_link_guard.enabled = 1u;
    errors = nav_runtime_config_validate(&cfg);
    CHECK((errors & NAV_CONFIG_ERROR_SWARM) != 0u);

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

    /* External fused state is the NavState authority and bypasses INS. */
    nav_runtime_config_default(&cfg);
    cfg.flight_state_source = NAV_FLIGHT_STATE_EXTERNAL;
    CHECK(nav_runtime_init(&runtime, &cfg) == NAV_CONFIG_ERROR_NONE);
    imu.timestamp_ms = 100u;
    input = stationary_input(&imu, cfg.nominal_dt_s);
    flight_state = make_flight_state(100u);
    input.flight_state = &flight_state;
    CHECK(nav_runtime_step(&runtime, &input) == 1u);
    CHECK(runtime.output.flight_state_source == NAV_FLIGHT_STATE_EXTERNAL);
    CHECK(runtime.output.nav.mode == EST_MODE_EXTERNAL);
    CHECK(runtime.output.nav.status == EST_TRACKING);
    CHECK(runtime.output.nav.validity == NAV_VALID_VALID);
    CHECK(runtime.output.nav.timestamp_ms == 100u);
    CHECK(fabsf(runtime.output.nav.pos.x - 1.0f) < 1e-5f);
    CHECK(fabsf(runtime.output.nav.vel.y + 0.2f) < 1e-5f);
    CHECK(fabsf(runtime.output.nav.yaw - 0.4f) < 1e-5f);

    /* An authoritative sample cannot be stale. */
    CHECK(nav_runtime_init(&runtime, &cfg) == NAV_CONFIG_ERROR_NONE);
    imu.timestamp_ms = 300u;
    input = stationary_input(&imu, cfg.nominal_dt_s);
    flight_state = make_flight_state(100u);
    input.flight_state = &flight_state;
    CHECK(nav_runtime_step(&runtime, &input) == 0u);
    CHECK((runtime.output.health.stale_source_mask &
           NAV_INPUT_SOURCE_FLIGHT_STATE) != 0u);
    CHECK(runtime.output.nav.validity == NAV_VALID_INVALID);
    CHECK(runtime.output.armed == 0u);

    /* Position and velocity are independently required. */
    CHECK(nav_runtime_init(&runtime, &cfg) == NAV_CONFIG_ERROR_NONE);
    imu.timestamp_ms = 400u;
    input = stationary_input(&imu, cfg.nominal_dt_s);
    flight_state = make_flight_state(400u);
    flight_state.valid_mask &= ~FLIGHT_STATE_VALID_POSITION;
    input.flight_state = &flight_state;
    CHECK(nav_runtime_step(&runtime, &input) == 0u);
    CHECK((runtime.output.health.invalid_source_mask &
           NAV_INPUT_SOURCE_FLIGHT_STATE) != 0u);

    CHECK(nav_runtime_init(&runtime, &cfg) == NAV_CONFIG_ERROR_NONE);
    imu.timestamp_ms = 410u;
    input = stationary_input(&imu, cfg.nominal_dt_s);
    flight_state = make_flight_state(410u);
    flight_state.valid_mask &= ~FLIGHT_STATE_VALID_VELOCITY;
    input.flight_state = &flight_state;
    CHECK(nav_runtime_step(&runtime, &input) == 0u);

    /* Non-finite values, unknown frames and invalid quaternions are rejected. */
    CHECK(nav_runtime_init(&runtime, &cfg) == NAV_CONFIG_ERROR_NONE);
    imu.timestamp_ms = 500u;
    input = stationary_input(&imu, cfg.nominal_dt_s);
    flight_state = make_flight_state(500u);
    flight_state.position_m.x = NAN;
    input.flight_state = &flight_state;
    CHECK(nav_runtime_step(&runtime, &input) == 0u);
    CHECK((runtime.output.health.nonfinite_source_mask &
           NAV_INPUT_SOURCE_FLIGHT_STATE) != 0u);

    CHECK(nav_runtime_init(&runtime, &cfg) == NAV_CONFIG_ERROR_NONE);
    imu.timestamp_ms = 510u;
    input = stationary_input(&imu, cfg.nominal_dt_s);
    flight_state = make_flight_state(510u);
    flight_state.frame = (FlightStateFrame)1;
    input.flight_state = &flight_state;
    CHECK(nav_runtime_step(&runtime, &input) == 0u);

    CHECK(nav_runtime_init(&runtime, &cfg) == NAV_CONFIG_ERROR_NONE);
    imu.timestamp_ms = 520u;
    input = stationary_input(&imu, cfg.nominal_dt_s);
    flight_state = make_flight_state(520u);
    flight_state.attitude_body_to_nav.w = 0.0f;
    flight_state.attitude_body_to_nav.x = 0.0f;
    flight_state.attitude_body_to_nav.y = 0.0f;
    flight_state.attitude_body_to_nav.z = 0.0f;
    input.flight_state = &flight_state;
    CHECK(nav_runtime_step(&runtime, &input) == 0u);

    /* Static ownership rejects either kind of competing state provider. */
    CHECK(nav_runtime_init(&runtime, &cfg) == NAV_CONFIG_ERROR_NONE);
    imu.timestamp_ms = 600u;
    input = stationary_input(&imu, cfg.nominal_dt_s);
    flight_state = make_flight_state(600u);
    odometry.pos = vec3_zero();
    odometry.vel = vec3_zero();
    odometry.yaw = 0.0f;
    odometry.yaw_rate = 0.0f;
    odometry.att = quat_identity();
    odometry.valid = 1u;
    odometry.timestamp_ms = 600u;
    input.flight_state = &flight_state;
    input.odometry = &odometry;
    CHECK(nav_runtime_step(&runtime, &input) == 0u);
    CHECK((runtime.output.health.invalid_source_mask &
           NAV_INPUT_SOURCE_ODOMETRY) != 0u);

    nav_runtime_config_default(&cfg);
    CHECK(nav_runtime_init(&runtime, &cfg) == NAV_CONFIG_ERROR_NONE);
    input = stationary_input(&imu, cfg.nominal_dt_s);
    input.flight_state = &flight_state;
    CHECK(nav_runtime_step(&runtime, &input) == 0u);
    CHECK((runtime.output.health.invalid_source_mask &
           NAV_INPUT_SOURCE_FLIGHT_STATE) != 0u);

    /* Localization loss is explicit and a newer valid state recovers. */
    nav_runtime_config_default(&cfg);
    cfg.flight_state_source = NAV_FLIGHT_STATE_EXTERNAL;
    CHECK(nav_runtime_init(&runtime, &cfg) == NAV_CONFIG_ERROR_NONE);
    imu.timestamp_ms = 700u;
    input = stationary_input(&imu, cfg.nominal_dt_s);
    flight_state = make_flight_state(700u);
    input.flight_state = &flight_state;
    CHECK(nav_runtime_step(&runtime, &input) == 1u);
    imu.timestamp_ms = 710u;
    input.timestamp_ms = 710u;
    flight_state.timestamp_ms = 710u;
    flight_state.valid = 0u;
    CHECK(nav_runtime_step(&runtime, &input) == 0u);
    CHECK(runtime.output.nav.status == EST_LOST);
    imu.timestamp_ms = 720u;
    input.timestamp_ms = 720u;
    flight_state.timestamp_ms = 720u;
    flight_state.position_m.x = 2.5f;
    flight_state.valid = 1u;
    CHECK(nav_runtime_step(&runtime, &input) == 1u);
    CHECK(runtime.output.nav.status == EST_TRACKING);
    CHECK(fabsf(runtime.output.nav.pos.x - 2.5f) < 1e-5f);

    nav_runtime_config_default(&cfg);
    cfg.flight_state_source = (NavFlightStateSource)99;
    CHECK((nav_runtime_config_validate(&cfg) &
           NAV_CONFIG_ERROR_STATE_SOURCE) != 0u);

    if (failures == 0) {
        printf("test_nav_runtime: PASS\n");
        return 0;
    }
    printf("test_nav_runtime: %d FAILURES\n", failures);
    return 1;
}
