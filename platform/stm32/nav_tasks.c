/* STM32 peripheral adapter. All navigation behavior lives in NavRuntime. */
#include "nav_tasks.h"
#include "nav_platform.h"

static void nav_app_emit_telemetry(NavApp *app, uint32_t timestamp_ms,
                                   uint8_t force)
{
#if NAV_APP_TELEMETRY_PERIOD_STEPS > 0u
    NavTelemetrySnapshot snapshot;
    uint8_t frame[NAV_TELEMETRY_FRAME_SIZE];
    if (!force) {
        app->telemetry_step_count++;
        if (app->telemetry_step_count < NAV_APP_TELEMETRY_PERIOD_STEPS) return;
    }
    app->telemetry_step_count = 0u;
    if (!nav_telemetry_capture(&snapshot, &app->runtime.output,
                               app->telemetry_sequence, timestamp_ms) ||
        nav_telemetry_encode(&snapshot, frame) != NAV_TELEMETRY_OK) {
        nav_log("navigation telemetry encode failed");
        return;
    }
    app->telemetry_sequence++;
    (void)nav_telemetry_write(frame, NAV_TELEMETRY_FRAME_SIZE);
#else
    (void)app;
    (void)timestamp_ms;
    (void)force;
#endif
}

static void nav_app_flush_events(NavApp *app)
{
    const NavEventLog *log = nav_runtime_event_log(&app->runtime);
    NavEventRecord record;
    uint16_t index;
    if (log == 0 || log->next_sequence == app->next_log_sequence) return;
    for (index = 0u; index < nav_event_log_count(log); index++) {
        if (nav_event_log_get(log, index, &record) &&
            record.sequence >= app->next_log_sequence) {
            nav_log(nav_log_event_code_name(record.code));
            app->next_log_sequence = record.sequence + 1u;
        }
    }
}

static void nav_app_disable_fcu(NavApp *app, uint32_t timestamp_ms)
{
    FcuSetpoint setpoint;
    fcu_setpoint_disable(app->fcu_command_sequence++, timestamp_ms, &setpoint);
    if (nav_fcu_setpoint_write(&setpoint) != 0) app->fcu_tx_drops++;
    nav_fcu_set_armed(0u);
}

static void nav_app_send_fcu(NavApp *app, uint32_t timestamp_ms)
{
    FcuSetpoint setpoint;
    if (!fcu_setpoint_from_nav(&app->runtime.output.guidance,
                               &app->runtime.output.control,
                               FCU_CONTROL_AUTO,
                               app->runtime.output.armed,
                               app->fcu_command_sequence++, timestamp_ms,
                               NAV_APP_FCU_COMMAND_VALIDITY_MS, &setpoint) ||
        nav_fcu_setpoint_write(&setpoint) != 0) {
        app->fcu_tx_drops++;
    }
}

uint32_t nav_app_init(NavApp *app, const NavAppConfig *cfg)
{
    if (app == 0) return NAV_CONFIG_ERROR_ARGUMENT;
    app->config_errors = nav_runtime_init(&app->runtime, cfg);
    app->next_log_sequence = 0u;
    app->telemetry_sequence = 0u;
    app->telemetry_step_count = 0u;
    app->fcu_tx_drops = 0u;
    app->fcu_command_sequence = 0u;
    if (app->config_errors != NAV_CONFIG_ERROR_NONE) {
        nav_log("navigation configuration invalid");
    }
    return app->config_errors;
}

void nav_app_step(NavApp *app, float dt)
{
    uint32_t time_ms;
    ImuSample imu;
    FlowFrame frame;
    PixelObs target_pixel;
    PixelObs home_pixel;
    DynamicObstacleSet obstacles;
    SwarmView swarm;
    NavMissionCommand mission_command;
    NavDockStatus dock_status;
    NavRuntimeInput input;
    float tof_height = -1.0f;
    uint8_t flow_valid;
    uint8_t target_valid;
    uint8_t home_valid;
    uint8_t obstacles_valid;
    uint8_t swarm_valid;

    if (app == 0) return;
    time_ms = nav_time_ms();
    if (app->config_errors != NAV_CONFIG_ERROR_NONE ||
        nav_imu_read(&imu) != 0) {
        nav_app_disable_fcu(app, time_ms);
        return;
    }

    (void)nav_tof_read(&tof_height);
    flow_valid = (nav_flow_read(&frame) == 0) ? 1u : 0u;
    target_valid = (nav_camera_target_read(&target_pixel) == 0) ? 1u : 0u;
    home_valid = (nav_camera_home_read(&home_pixel) == 0) ? 1u : 0u;
    obstacle_set_init(&obstacles);
    obstacles_valid = (nav_local_obstacles_read(&obstacles) == 0) ? 1u : 0u;
    swarm_view_init(&swarm, app->runtime.cfg.self_agent_id);
    swarm_valid = (nav_swarm_view_read(&swarm) == 0) ? 1u : 0u;

    mission_command.start = 0u;
    mission_command.request_return = 0u;
    mission_command.request_emergency = 0u;
    (void)nav_mission_command_read(&mission_command);
    dock_status.contact = 0u;
    dock_status.charging = 0u;
    dock_status.wireless_charge_ready = 0u;
    (void)nav_dock_status_read(&dock_status);

    input.timestamp_ms = time_ms;
    input.imu = &imu;
    input.flow = flow_valid ? &frame : 0;
    input.odometry = 0;
    input.tof_height = tof_height;
    input.target_pixel = target_valid ? &target_pixel : 0;
    input.home_pixel = home_valid ? &home_pixel : 0;
    input.obstacles = obstacles_valid ? &obstacles : 0;
    input.swarm = swarm_valid ? &swarm : 0;
    input.start_command = mission_command.start;
    input.request_return = mission_command.request_return;
    input.request_emergency = mission_command.request_emergency;
    input.dock_contact = dock_status.contact;
    input.charging_detected = dock_status.charging;
    input.wireless_charge_ready = dock_status.wireless_charge_ready;
    input.dt = dt;

    if (!nav_runtime_step(&app->runtime, &input)) {
        nav_app_flush_events(app);
        nav_app_emit_telemetry(app, time_ms, 1u);
        nav_app_disable_fcu(app, time_ms);
        return;
    }

    nav_app_flush_events(app);
    nav_app_emit_telemetry(app, time_ms, 0u);
    nav_swarm_state_send(&app->runtime.output.swarm_self);
    nav_app_send_fcu(app, time_ms);
    nav_fcu_set_armed(app->runtime.output.armed);
}
