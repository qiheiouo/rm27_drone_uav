/* Loopback-only PX4 SITL runner for the existing NavApp control path. */
#define _WIN32_WINNT 0x0600

#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET NavSocket;
#define NAV_INVALID_SOCKET INVALID_SOCKET
#define nav_socket_close closesocket
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
typedef int NavSocket;
#define NAV_INVALID_SOCKET (-1)
#define nav_socket_close close
#endif

#include "nav_tasks.h"
#include "nav_platform.h"
#include "px4_mavlink_adapter.h"

#define RUNNER_PERIOD_MS 10u
#define OFFBOARD_PRESTREAM_MS 1100u
#define COMMAND_RETRY_MS 1000u
#define HEARTBEAT_PERIOD_MS 1000u
#define PX4_SITL_CONTROLLER_SATURATION_TIMEOUT_S 8.0f

typedef struct {
    NavSocket socket_fd;
    struct sockaddr_in peer;
    Px4MavlinkAdapter adapter;
    NavApp app;
    FILE *log_file;
    uint32_t now_ms;
    uint32_t started_ms;
    uint32_t duration_ms;
    uint32_t first_active_setpoint_ms;
    uint32_t last_active_setpoint_ms;
    uint32_t last_heartbeat_tx_ms;
    uint32_t last_stream_request_ms;
    uint32_t last_mode_request_ms;
    uint32_t last_arm_request_ms;
    uint32_t last_report_ms;
    uint32_t last_nav_step_ms;
    uint32_t last_nav_state_sample_ms;
    uint32_t last_nav_imu_sample_ms;
    uint32_t inject_state_loss_at_ms;
    uint32_t inject_state_loss_duration_ms;
    uint32_t inject_link_loss_at_ms;
    uint32_t inject_link_loss_duration_ms;
    uint32_t reauthorize_at_ms;
    uint32_t setpoints_sent;
    uint32_t packets_received;
    uint32_t packets_dropped;
    uint16_t authorization_sequence;
    uint8_t peer_known;
    uint8_t stream_requests_sent;
    uint8_t start_requested;
    uint8_t authorization_requested;
    uint8_t authorization_decision_issued;
    uint8_t shutdown_requested;
    uint8_t shutdown_decision_issued;
    uint8_t arm_requested;
    uint8_t host_authorized;
    uint8_t ever_host_authorized;
    uint8_t ever_px4_armed;
    uint8_t exit_offboard_pending;
    uint8_t nav_inputs_valid;
    uint8_t state_loss_reported;
    uint8_t state_recovery_reported;
    uint8_t reauthorization_reissued;
    uint8_t link_loss_reported;
    uint8_t link_recovery_reported;
    uint8_t probe_offboard_before_prestream;
    uint8_t probe_offboard_issued;
    uint8_t probe_arm_issued;
} Px4SitlRunner;

static Px4SitlRunner *g_runner;
static volatile sig_atomic_t g_stop_requested;

static uint64_t monotonic_ms(void)
{
#ifdef _WIN32
    return (uint64_t)GetTickCount64();
#else
    struct timespec timestamp;
    if (clock_gettime(CLOCK_MONOTONIC, &timestamp) != 0) return 0u;
    return (uint64_t)timestamp.tv_sec * 1000u +
        (uint64_t)timestamp.tv_nsec / 1000000u;
#endif
}

static void sleep_ms(uint32_t milliseconds)
{
#ifdef _WIN32
    Sleep(milliseconds);
#else
    struct timespec delay;
    delay.tv_sec = milliseconds / 1000u;
    delay.tv_nsec = (long)(milliseconds % 1000u) * 1000000L;
    (void)nanosleep(&delay, 0);
#endif
}

static uint8_t runner_state_loss_active(const Px4SitlRunner *runner)
{
    uint32_t elapsed_ms;
    if (runner == 0 || runner->inject_state_loss_duration_ms == 0u) {
        return 0u;
    }
    elapsed_ms = (uint32_t)(runner->now_ms - runner->started_ms);
    return elapsed_ms >= runner->inject_state_loss_at_ms &&
        elapsed_ms < runner->inject_state_loss_at_ms +
            runner->inject_state_loss_duration_ms ? 1u : 0u;
}

static uint8_t runner_link_loss_active(const Px4SitlRunner *runner)
{
    uint32_t elapsed_ms;
    if (runner == 0 || runner->inject_link_loss_duration_ms == 0u) {
        return 0u;
    }
    elapsed_ms = (uint32_t)(runner->now_ms - runner->started_ms);
    return elapsed_ms >= runner->inject_link_loss_at_ms &&
        elapsed_ms < runner->inject_link_loss_at_ms +
            runner->inject_link_loss_duration_ms ? 1u : 0u;
}

static void stop_handler(int signal_number)
{
    (void)signal_number;
    g_stop_requested = 1;
}

static uint8_t is_loopback(const struct sockaddr_in *address)
{
    uint32_t host_address;
    if (address == 0 || address->sin_family != AF_INET) return 0u;
    host_address = ntohl(address->sin_addr.s_addr);
    return (host_address >> 24) == 127u ? 1u : 0u;
}

static int socket_set_nonblocking(NavSocket socket_fd)
{
#ifdef _WIN32
    u_long enabled = 1u;
    return ioctlsocket(socket_fd, FIONBIO, &enabled) == 0 ? 0 : -1;
#else
    int flags = fcntl(socket_fd, F_GETFL, 0);
    if (flags < 0) return -1;
    return fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK) == 0 ? 0 : -1;
#endif
}

static int socket_open_loopback(NavSocket *socket_out, uint16_t port)
{
    NavSocket socket_fd;
    struct sockaddr_in local_address;
#ifdef _WIN32
    WSADATA winsock_data;
    if (WSAStartup(MAKEWORD(2, 2), &winsock_data) != 0) return -1;
#endif
    socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_fd == NAV_INVALID_SOCKET) return -1;
    memset(&local_address, 0, sizeof(local_address));
    local_address.sin_family = AF_INET;
    local_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    local_address.sin_port = htons(port);
    if (bind(socket_fd, (const struct sockaddr *)&local_address,
             sizeof(local_address)) != 0 ||
        socket_set_nonblocking(socket_fd) != 0) {
        nav_socket_close(socket_fd);
#ifdef _WIN32
        WSACleanup();
#endif
        return -1;
    }
    *socket_out = socket_fd;
    return 0;
}

static void socket_shutdown(NavSocket socket_fd)
{
    if (socket_fd != NAV_INVALID_SOCKET) nav_socket_close(socket_fd);
#ifdef _WIN32
    WSACleanup();
#endif
}

static int runner_send_message(Px4SitlRunner *runner,
                               const mavlink_message_t *message)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t length;
    int sent;
    if (runner == 0 || message == 0 || !runner->peer_known) return -1;
    if (runner_link_loss_active(runner)) {
        runner->packets_dropped++;
        return 0;
    }
    length = mavlink_msg_to_send_buffer(buffer, message);
    sent = (int)sendto(runner->socket_fd, (const char *)buffer, (int)length,
                       0, (const struct sockaddr *)&runner->peer,
                       sizeof(runner->peer));
    return sent == (int)length ? 0 : -1;
}

static void runner_receive(Px4SitlRunner *runner)
{
    uint8_t bytes[2048];
    for (;;) {
        struct sockaddr_in source;
#ifdef _WIN32
        int source_size = (int)sizeof(source);
#else
        socklen_t source_size = (socklen_t)sizeof(source);
#endif
        int received = (int)recvfrom(
            runner->socket_fd, (char *)bytes, (int)sizeof(bytes), 0,
            (struct sockaddr *)&source, &source_size);
        int index;
        if (received < 0) {
#ifdef _WIN32
            if (WSAGetLastError() == WSAEWOULDBLOCK) return;
#else
            if (errno == EAGAIN || errno == EWOULDBLOCK) return;
#endif
            return;
        }
        if (!is_loopback(&source)) continue;
        if (runner_link_loss_active(runner)) {
            if (!runner->link_loss_reported) {
                fprintf(stderr, "fault: injected-mavlink-loss\n");
                runner->link_loss_reported = 1u;
            }
            runner->packets_dropped++;
            continue;
        }
        if (runner->link_loss_reported && !runner->link_recovery_reported) {
            fprintf(stderr, "fault: injected-mavlink-recovered\n");
            runner->link_recovery_reported = 1u;
        }
        for (index = 0; index < received; index++) {
            mavlink_message_t message;
            mavlink_status_t status;
            if (mavlink_parse_char(MAVLINK_COMM_0, bytes[index], &message,
                                   &status)) {
                Px4MavlinkIngestResult result = px4_mavlink_ingest(
                    &runner->adapter, &message, runner->now_ms);
                runner->packets_received++;
                if (result == PX4_MAVLINK_ACCEPTED &&
                    message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                    runner->peer = source;
                    runner->peer_known = 1u;
                }
            }
        }
    }
}

static void runner_send_heartbeat(Px4SitlRunner *runner)
{
    mavlink_message_t message;
    if (!runner->peer_known ||
        (uint32_t)(runner->now_ms - runner->last_heartbeat_tx_ms) <
            HEARTBEAT_PERIOD_MS) {
        return;
    }
    if (px4_mavlink_make_heartbeat(&runner->adapter, &message) &&
        runner_send_message(runner, &message) == 0) {
        runner->last_heartbeat_tx_ms = runner->now_ms;
    }
}

static void runner_request_stream(Px4SitlRunner *runner,
                                  uint32_t message_id,
                                  int32_t interval_us)
{
    mavlink_message_t message;
    if (px4_mavlink_make_message_interval_request(
            &runner->adapter, message_id, interval_us, &message)) {
        (void)runner_send_message(runner, &message);
    }
}

static void runner_request_state_streams(Px4SitlRunner *runner)
{
    runner_request_stream(runner, MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20000);
    runner_request_stream(runner, MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 10000);
    runner_request_stream(runner, MAVLINK_MSG_ID_HIGHRES_IMU, 10000);
    runner_request_stream(runner, MAVLINK_MSG_ID_ESTIMATOR_STATUS, 100000);
    runner_request_stream(runner, MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 500000);
}

static void runner_update_control_mode(Px4SitlRunner *runner)
{
    Px4VehicleFeedback feedback;
    mavlink_message_t message;
    uint8_t stream_continuous;
    px4_mavlink_get_feedback(&runner->adapter, runner->now_ms, &feedback);
    if (feedback.armed) runner->ever_px4_armed = 1u;

    if (runner->exit_offboard_pending) {
        if (feedback.link_active &&
            (uint32_t)(runner->now_ms - runner->last_mode_request_ms) >=
                COMMAND_RETRY_MS &&
            px4_mavlink_make_hold_request(&runner->adapter, &message)) {
            (void)runner_send_message(runner, &message);
            runner->last_mode_request_ms = runner->now_ms;
        }
        if (!feedback.offboard_active) runner->exit_offboard_pending = 0u;
        return;
    }

    if (runner->probe_offboard_before_prestream) {
        if (!runner->probe_offboard_issued && feedback.link_active &&
            px4_mavlink_make_offboard_request(&runner->adapter, &message)) {
            (void)runner_send_message(runner, &message);
            runner->last_mode_request_ms = runner->now_ms;
            runner->probe_offboard_issued = 1u;
            fprintf(stderr, "fault: offboard-request-before-prestream\n");
        }
        if (feedback.offboard_active && !runner->probe_arm_issued &&
            px4_mavlink_make_arm_request(&runner->adapter, 1u, &message)) {
            (void)runner_send_message(runner, &message);
            runner->probe_arm_issued = 1u;
            fprintf(stderr, "fault: arm-request-without-offboard-stream\n");
        }
        return;
    }

    stream_continuous = runner->host_authorized &&
        runner->first_active_setpoint_ms != 0u &&
        (uint32_t)(runner->now_ms - runner->last_active_setpoint_ms) <= 50u &&
        (uint32_t)(runner->now_ms - runner->first_active_setpoint_ms) >=
            OFFBOARD_PRESTREAM_MS;
    if (!stream_continuous || !feedback.link_active) return;

    if (!feedback.offboard_active &&
        (uint32_t)(runner->now_ms - runner->last_mode_request_ms) >=
            COMMAND_RETRY_MS &&
        px4_mavlink_make_offboard_request(&runner->adapter, &message)) {
        (void)runner_send_message(runner, &message);
        runner->last_mode_request_ms = runner->now_ms;
        return;
    }
    if (feedback.offboard_active && runner->arm_requested &&
        !feedback.armed && !runner->ever_px4_armed &&
        (uint32_t)(runner->now_ms - runner->last_arm_request_ms) >=
            COMMAND_RETRY_MS &&
        px4_mavlink_make_arm_request(&runner->adapter, 1u, &message)) {
        (void)runner_send_message(runner, &message);
        runner->last_arm_request_ms = runner->now_ms;
    }
}

static void runner_step_navigation(Px4SitlRunner *runner)
{
    FlightState state;
    ImuSample imu;
    uint8_t state_valid;
    uint8_t imu_valid;
    float dt_s;

    state_valid = px4_mavlink_get_flight_state(
        &runner->adapter, runner->now_ms, &state, 0);
    imu_valid = px4_mavlink_get_imu(
        &runner->adapter, runner->now_ms, &imu, 0);
    if (state_valid && imu_valid) {
        if (runner->nav_inputs_valid &&
            state.timestamp_ms == runner->last_nav_state_sample_ms) {
            return;
        }
        if (runner->nav_inputs_valid &&
            imu.timestamp_ms == runner->last_nav_imu_sample_ms) {
            return;
        }
        dt_s = !runner->nav_inputs_valid || runner->last_nav_step_ms == 0u
            ? (float)RUNNER_PERIOD_MS / 1000.0f
            : (float)(runner->now_ms - runner->last_nav_step_ms) / 1000.0f;
        runner->last_nav_step_ms = runner->now_ms;
        runner->last_nav_state_sample_ms = state.timestamp_ms;
        runner->last_nav_imu_sample_ms = imu.timestamp_ms;
        runner->nav_inputs_valid = 1u;
        nav_app_step(&runner->app, dt_s);
        return;
    }

    /* A validity transition must reach NavApp immediately so the existing
     * final authorization gate revokes motion. Avoid repeatedly submitting
     * the same invalid input while the link remains unavailable. */
    if (runner->nav_inputs_valid) {
        runner->nav_inputs_valid = 0u;
        runner->last_nav_step_ms = runner->now_ms;
        nav_app_step(&runner->app,
                     (float)RUNNER_PERIOD_MS / 1000.0f);
    }
}

static void runner_report(Px4SitlRunner *runner)
{
    Px4VehicleFeedback feedback;
    FlightState state;
    uint32_t age_ms = 0u;
    uint8_t state_valid;
    if ((uint32_t)(runner->now_ms - runner->last_report_ms) < 1000u) return;
    runner->last_report_ms = runner->now_ms;
    px4_mavlink_get_feedback(&runner->adapter, runner->now_ms, &feedback);
    state_valid = px4_mavlink_get_flight_state(
        &runner->adapter, runner->now_ms, &state, &age_ms);
    printf("t=%lu link=%u state=%u age=%lu host_auth=%u sent=%lu "
           "offboard=%u armed=%u system=%u est_seen=%u est_flags=%lu "
           "est_age=%lu imu_seen=%u imu_age=%lu state_mask=%lu\n",
           (unsigned long)(runner->now_ms - runner->started_ms),
           feedback.link_active, state_valid, (unsigned long)age_ms,
           runner->host_authorized, (unsigned long)runner->setpoints_sent,
           feedback.offboard_active, feedback.armed, feedback.system_status,
           runner->adapter.estimator_seen,
           (unsigned long)runner->adapter.estimator_flags,
           (unsigned long)(runner->adapter.estimator_seen
               ? runner->now_ms - runner->adapter.estimator_received_ms : 0u),
           runner->adapter.imu_seen,
           (unsigned long)(runner->adapter.imu_seen
               ? runner->now_ms - runner->adapter.imu_received_ms : 0u),
           (unsigned long)runner->adapter.state.valid_mask);
    if (runner->log_file != 0) fflush(runner->log_file);
}

static void runner_log_sample(Px4SitlRunner *runner)
{
    Px4VehicleFeedback feedback;
    FlightState state;
    const NavRuntimeOutput *output;
    uint32_t age_ms = 0u;
    uint8_t state_valid;
    float tracking_error = -1.0f;
    if (runner->log_file == 0) return;
    px4_mavlink_get_feedback(&runner->adapter, runner->now_ms, &feedback);
    state_valid = px4_mavlink_get_flight_state(
        &runner->adapter, runner->now_ms, &state, &age_ms);
    output = &runner->app.runtime.output;
    if (state_valid && output->step_valid &&
        (output->mission.guidance == GM_TAKEOFF ||
         output->mission.guidance == GM_WAYPOINT)) {
        Vec3f target = output->mission.guidance == GM_TAKEOFF
            ? output->mission.hold_pos
            : output->mission.current_waypoint.pos;
        tracking_error = vec3_dist(state.position_m, target);
    }
    fprintf(runner->log_file,
        "%lu,%u,%u,%lu,%lu,%u,%u,%u,%u,%lu,%u,%u,%u,%u,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u,%u\n",
        (unsigned long)(runner->now_ms - runner->started_ms),
        feedback.link_active, state_valid, (unsigned long)age_ms,
        (unsigned long)(state_valid ? state.timestamp_ms : 0u),
        runner->host_authorized, runner->authorization_sequence,
        runner_state_loss_active(runner),
        runner_link_loss_active(runner),
        (unsigned long)runner->setpoints_sent,
        feedback.offboard_active, feedback.armed,
        feedback.last_command_ack, feedback.last_command_result,
        state_valid ? state.position_m.x : 0.0f,
        state_valid ? state.position_m.y : 0.0f,
        state_valid ? state.position_m.z : 0.0f,
        state_valid ? state.velocity_mps.x : 0.0f,
        state_valid ? state.velocity_mps.y : 0.0f,
        state_valid ? state.velocity_mps.z : 0.0f,
        output->control.velocity_cmd.x,
        output->control.velocity_cmd.y,
        output->control.velocity_cmd.z,
        tracking_error, output->mission.state, output->step_valid,
        output->armed);
}

uint32_t nav_time_ms(void)
{
    return g_runner != 0 ? g_runner->now_ms : 0u;
}

int nav_imu_read(ImuSample *out)
{
    return g_runner != 0 && px4_mavlink_get_imu(
        &g_runner->adapter, g_runner->now_ms, out, 0) ? 0 : -1;
}

int nav_flight_state_read(FlightState *out)
{
    if (runner_state_loss_active(g_runner)) {
        if (!g_runner->state_loss_reported) {
            fprintf(stderr, "fault: injected-flight-state-loss\n");
            g_runner->state_loss_reported = 1u;
        }
        return -1;
    }
    if (g_runner != 0 && g_runner->state_loss_reported &&
        !g_runner->state_recovery_reported) {
        fprintf(stderr, "fault: injected-flight-state-recovered\n");
        g_runner->state_recovery_reported = 1u;
    }
    return g_runner != 0 && px4_mavlink_get_flight_state(
        &g_runner->adapter, g_runner->now_ms, out, 0) ? 0 : -1;
}

int nav_tof_read(float *height_m)
{
    (void)height_m;
    return -1;
}

int nav_flow_read(FlowFrame *out)
{
    (void)out;
    return -1;
}

int nav_camera_target_read(PixelObs *out)
{
    (void)out;
    return -1;
}

int nav_camera_home_read(PixelObs *out)
{
    (void)out;
    return -1;
}

int nav_local_obstacles_read(DynamicObstacleSet *out)
{
    (void)out;
    return -1;
}

int nav_mission_command_read(NavMissionCommand *out)
{
    uint32_t elapsed_ms;
    if (g_runner == 0 || out == 0) return -1;
    memset(out, 0, sizeof(*out));
    out->start = g_runner->start_requested;
    if (g_runner->shutdown_requested) {
        if (!g_runner->shutdown_decision_issued) {
            g_runner->authorization_sequence++;
            if (g_runner->authorization_sequence == 0u) {
                g_runner->authorization_sequence = 1u;
            }
            g_runner->shutdown_decision_issued = 1u;
        }
        out->motion_authorization_request = 0u;
        out->motion_authorization_sequence =
            g_runner->authorization_sequence;
        return 0;
    }
    elapsed_ms = (uint32_t)(g_runner->now_ms - g_runner->started_ms);
    if (g_runner->reauthorize_at_ms != 0u &&
        !g_runner->reauthorization_reissued &&
        elapsed_ms >= g_runner->reauthorize_at_ms) {
        g_runner->authorization_sequence++;
        if (g_runner->authorization_sequence == 0u) {
            g_runner->authorization_sequence = 1u;
        }
        g_runner->reauthorization_reissued = 1u;
        fprintf(stderr, "fault: explicit-reauthorization-sequence=%u\n",
                g_runner->authorization_sequence);
    }
    if (g_runner->authorization_requested &&
        !g_runner->authorization_decision_issued &&
        g_runner->app.runtime.output.armed) {
        g_runner->authorization_sequence++;
        if (g_runner->authorization_sequence == 0u) {
            g_runner->authorization_sequence = 1u;
        }
        g_runner->authorization_decision_issued = 1u;
    }
    if (g_runner->authorization_decision_issued) {
        out->motion_authorization_request = 1u;
        out->motion_authorization_sequence =
            g_runner->authorization_sequence;
    }
    return 0;
}

int nav_dock_status_read(NavDockStatus *out)
{
    if (out != 0) memset(out, 0, sizeof(*out));
    return 0;
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

int nav_fcu_setpoint_write(const FcuSetpoint *setpoint)
{
    mavlink_message_t message;
    Px4SetpointBuildResult result;
    if (g_runner == 0) return -1;
    if (g_runner->probe_offboard_before_prestream) return 0;
    result = px4_mavlink_make_velocity_setpoint(
        &g_runner->adapter, setpoint, g_runner->now_ms, &message);
    if (result == PX4_SETPOINT_DISABLED) return 0;
    if (result != PX4_SETPOINT_READY ||
        runner_send_message(g_runner, &message) != 0) {
        return -1;
    }
    if (g_runner->last_active_setpoint_ms == 0u ||
        (uint32_t)(g_runner->now_ms -
                   g_runner->last_active_setpoint_ms) > 50u) {
        g_runner->first_active_setpoint_ms = g_runner->now_ms;
    }
    g_runner->last_active_setpoint_ms = g_runner->now_ms;
    g_runner->setpoints_sent++;
    return 0;
}

void nav_fcu_set_motion_authorized(uint8_t authorized)
{
    if (g_runner == 0) return;
    if (!authorized && g_runner->host_authorized &&
        g_runner->ever_host_authorized) {
        g_runner->exit_offboard_pending = 1u;
        g_runner->first_active_setpoint_ms = 0u;
        g_runner->last_active_setpoint_ms = 0u;
    }
    g_runner->host_authorized = authorized;
    if (authorized) g_runner->ever_host_authorized = 1u;
}

void nav_log(const char *message)
{
    if (message != 0) fprintf(stderr, "nav: %s\n", message);
}

int nav_telemetry_write(const uint8_t *frame, uint16_t size)
{
    (void)frame;
    (void)size;
    return 0;
}

static void usage(const char *program)
{
    printf("Usage: %s --sitl --authorize-motion --arm "
           "[--port 14540] [--duration-s 30] [--log FILE] "
           "[--inject-state-loss-at-s N "
           "--inject-state-loss-duration-s N "
           "--reauthorize-at-s N] "
           "[--inject-link-loss-at-s N "
           "--inject-link-loss-duration-s N] "
           "[--probe-offboard-before-prestream]\n", program);
    printf("Loopback only. This executable refuses non-SITL/remote endpoints.\n");
}

static int parse_u32(const char *text, uint32_t minimum, uint32_t maximum,
                     uint32_t *value)
{
    char *end = 0;
    unsigned long parsed;
    if (text == 0 || value == 0) return -1;
    parsed = strtoul(text, &end, 10);
    if (*text == '\0' || end == 0 || *end != '\0' ||
        parsed < minimum || parsed > maximum) {
        return -1;
    }
    *value = (uint32_t)parsed;
    return 0;
}

int main(int argc, char **argv)
{
    Px4SitlRunner runner;
    Px4MavlinkConfig adapter_cfg;
    NavAppConfig nav_cfg;
    const char *log_path = 0;
    uint32_t bind_port = 14540u;
    uint32_t duration_seconds = 30u;
    uint32_t parsed_seconds;
    uint64_t monotonic_start;
    uint64_t next_step;
    uint8_t sitl_confirmed = 0u;
    int index;

    memset(&runner, 0, sizeof(runner));
    runner.socket_fd = NAV_INVALID_SOCKET;
    for (index = 1; index < argc; index++) {
        if (strcmp(argv[index], "--sitl") == 0) {
            sitl_confirmed = 1u;
        } else if (strcmp(argv[index], "--authorize-motion") == 0) {
            runner.authorization_requested = 1u;
        } else if (strcmp(argv[index], "--arm") == 0) {
            runner.arm_requested = 1u;
        } else if (strcmp(argv[index], "--port") == 0 && index + 1 < argc) {
            if (parse_u32(argv[++index], 1024u, 65535u, &bind_port) != 0) {
                usage(argv[0]);
                return 2;
            }
        } else if (strcmp(argv[index], "--duration-s") == 0 &&
                   index + 1 < argc) {
            if (parse_u32(argv[++index], 1u, 3600u, &duration_seconds) != 0) {
                usage(argv[0]);
                return 2;
            }
        } else if (strcmp(argv[index], "--log") == 0 && index + 1 < argc) {
            log_path = argv[++index];
        } else if (strcmp(argv[index], "--inject-state-loss-at-s") == 0 &&
                   index + 1 < argc) {
            if (parse_u32(argv[++index], 1u, 3600u, &parsed_seconds) != 0) {
                usage(argv[0]);
                return 2;
            }
            runner.inject_state_loss_at_ms = parsed_seconds * 1000u;
        } else if (strcmp(argv[index],
                          "--inject-state-loss-duration-s") == 0 &&
                   index + 1 < argc) {
            if (parse_u32(argv[++index], 1u, 3600u, &parsed_seconds) != 0) {
                usage(argv[0]);
                return 2;
            }
            runner.inject_state_loss_duration_ms = parsed_seconds * 1000u;
        } else if (strcmp(argv[index], "--reauthorize-at-s") == 0 &&
                   index + 1 < argc) {
            if (parse_u32(argv[++index], 1u, 3600u, &parsed_seconds) != 0) {
                usage(argv[0]);
                return 2;
            }
            runner.reauthorize_at_ms = parsed_seconds * 1000u;
        } else if (strcmp(argv[index], "--inject-link-loss-at-s") == 0 &&
                   index + 1 < argc) {
            if (parse_u32(argv[++index], 1u, 3600u, &parsed_seconds) != 0) {
                usage(argv[0]);
                return 2;
            }
            runner.inject_link_loss_at_ms = parsed_seconds * 1000u;
        } else if (strcmp(argv[index],
                          "--inject-link-loss-duration-s") == 0 &&
                   index + 1 < argc) {
            if (parse_u32(argv[++index], 1u, 3600u, &parsed_seconds) != 0) {
                usage(argv[0]);
                return 2;
            }
            runner.inject_link_loss_duration_ms = parsed_seconds * 1000u;
        } else if (strcmp(argv[index],
                          "--probe-offboard-before-prestream") == 0) {
            runner.probe_offboard_before_prestream = 1u;
        } else if (strcmp(argv[index], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            usage(argv[0]);
            return 2;
        }
    }
    if (!sitl_confirmed || !runner.authorization_requested ||
        !runner.arm_requested) {
        fprintf(stderr, "Refusing to run: all three explicit SITL safety "
                        "flags are required.\n");
        usage(argv[0]);
        return 2;
    }
    if ((runner.inject_state_loss_at_ms == 0u) !=
            (runner.inject_state_loss_duration_ms == 0u) ||
        (runner.inject_link_loss_at_ms == 0u) !=
            (runner.inject_link_loss_duration_ms == 0u) ||
        (runner.reauthorize_at_ms != 0u &&
         (runner.inject_state_loss_duration_ms == 0u ||
          runner.reauthorize_at_ms <= runner.inject_state_loss_at_ms +
              runner.inject_state_loss_duration_ms))) {
        fprintf(stderr, "Invalid fault schedule: state-loss time/duration "
                        "must be paired and reauthorization must follow "
                        "recovery.\n");
        usage(argv[0]);
        return 2;
    }
    runner.duration_ms = duration_seconds * 1000u;
    if (socket_open_loopback(&runner.socket_fd, (uint16_t)bind_port) != 0) {
        fprintf(stderr, "Cannot bind loopback UDP port %lu.\n",
                (unsigned long)bind_port);
        return 3;
    }
    if (log_path != 0) {
        runner.log_file = fopen(log_path, "w");
        if (runner.log_file == 0) {
            fprintf(stderr, "Cannot open log file: %s\n", log_path);
            socket_shutdown(runner.socket_fd);
            return 3;
        }
        fputs("elapsed_ms,link_active,state_valid,state_age_ms,state_sample_ms,"
              "host_authorized,authorization_sequence,injected_state_loss,"
              "injected_link_loss,setpoints_sent,offboard_active,px4_armed,"
              "last_ack_command,last_ack_result,pos_x,pos_y,pos_z,vel_x,vel_y,"
              "vel_z,cmd_vx,cmd_vy,cmd_vz,tracking_error_m,mission_state,"
              "step_valid,nav_motion_state\n", runner.log_file);
    }

    px4_mavlink_config_default(&adapter_cfg);
    px4_mavlink_init(&runner.adapter, &adapter_cfg);
    nav_runtime_config_default(&nav_cfg);
    nav_cfg.flight_state_source = NAV_FLIGHT_STATE_EXTERNAL;
    nav_cfg.mission.takeoff_alt_m = 1.0f;
    nav_cfg.mission.cruise_speed_mps = 0.4f;
    nav_cfg.ctrl.max_vel = 0.5f;
    nav_cfg.ctrl.max_accel = 1.0f;
    nav_cfg.safety.geofence_radius_m = 3.0f;
    nav_cfg.safety.geofence_max_alt_m = 2.0f;
    /* PX4 SITL defaults use 1 s motor spool-up plus a 3 s takeoff ramp. The
     * stock x500 then needs about 0.7 s above takeoff thrust to break ground.
     * Keep the protection active while leaving margin for that sequence. */
    nav_cfg.safety.controller_saturation_timeout_s =
        PX4_SITL_CONTROLLER_SATURATION_TIMEOUT_S;
    wq_init(&nav_cfg.outbound_route);
    (void)wq_push(&nav_cfg.outbound_route, vec3(1.0f, 0.0f, 1.0f), 0.4f);
    wq_init(&nav_cfg.search_route);
    (void)wq_push(&nav_cfg.search_route, vec3(1.0f, 0.0f, 1.0f), 0.2f);

    g_runner = &runner;
    if (nav_app_init(&runner.app, &nav_cfg) != NAV_CONFIG_ERROR_NONE) {
        fprintf(stderr, "Navigation configuration rejected.\n");
        if (runner.log_file != 0) fclose(runner.log_file);
        socket_shutdown(runner.socket_fd);
        return 4;
    }
    runner.start_requested = 1u;
    runner.last_mode_request_ms = UINT32_MAX - COMMAND_RETRY_MS + 1u;
    runner.last_arm_request_ms = UINT32_MAX - COMMAND_RETRY_MS + 1u;
    signal(SIGINT, stop_handler);
    monotonic_start = monotonic_ms();
    next_step = monotonic_start;
    runner.started_ms = (uint32_t)monotonic_start;
    printf("Waiting for PX4 SITL on 127.0.0.1:%lu; no remote packets accepted.\n",
           (unsigned long)bind_port);

    while (!g_stop_requested) {
        uint64_t current = monotonic_ms();
        runner.now_ms = (uint32_t)current;
        runner_receive(&runner);
        if (runner.peer_known &&
            (!runner.stream_requests_sent ||
             (uint32_t)(runner.now_ms - runner.last_stream_request_ms) >=
                 5000u)) {
            runner_request_state_streams(&runner);
            runner.stream_requests_sent = 1u;
            runner.last_stream_request_ms = runner.now_ms;
        }
        runner_send_heartbeat(&runner);
        if (runner.peer_known) {
            runner_step_navigation(&runner);
            runner_update_control_mode(&runner);
        }
        runner_log_sample(&runner);
        runner_report(&runner);
        if ((uint32_t)(runner.now_ms - runner.started_ms) >=
            runner.duration_ms) {
            break;
        }
        next_step += RUNNER_PERIOD_MS;
        current = monotonic_ms();
        if (next_step > current) sleep_ms((uint32_t)(next_step - current));
        else next_step = current;
    }

    if (runner.ever_host_authorized) {
        Px4VehicleFeedback feedback;
        mavlink_message_t message;
        uint32_t shutdown_started_ms;
        runner.shutdown_requested = 1u;
        shutdown_started_ms = (uint32_t)monotonic_ms();
        do {
            runner.now_ms = (uint32_t)monotonic_ms();
            runner_receive(&runner);
            runner_step_navigation(&runner);
            runner_log_sample(&runner);
            if (runner.shutdown_decision_issued &&
                !runner.host_authorized) {
                break;
            }
            sleep_ms(RUNNER_PERIOD_MS);
        } while ((uint32_t)(runner.now_ms - shutdown_started_ms) < 250u);
        if (px4_mavlink_make_hold_request(&runner.adapter, &message)) {
            (void)runner_send_message(&runner, &message);
        }
        shutdown_started_ms = (uint32_t)monotonic_ms();
        do {
            runner.now_ms = (uint32_t)monotonic_ms();
            runner_receive(&runner);
            runner_send_heartbeat(&runner);
            runner_update_control_mode(&runner);
            runner_log_sample(&runner);
            px4_mavlink_get_feedback(
                &runner.adapter, runner.now_ms, &feedback);
            if (feedback.link_active && !feedback.offboard_active) break;
            sleep_ms(RUNNER_PERIOD_MS);
        } while ((uint32_t)(runner.now_ms - shutdown_started_ms) < 2500u);
        px4_mavlink_get_feedback(&runner.adapter, runner.now_ms, &feedback);
        printf("Shutdown feedback: link=%u offboard=%u armed=%u "
               "system=%u.\n", feedback.link_active,
               feedback.offboard_active, feedback.armed,
               feedback.system_status);
    }
    printf("Stopped: packets=%lu dropped=%lu setpoints=%lu.\n",
           (unsigned long)runner.packets_received,
           (unsigned long)runner.packets_dropped,
           (unsigned long)runner.setpoints_sent);
    if (runner.log_file != 0) fclose(runner.log_file);
    socket_shutdown(runner.socket_fd);
    g_runner = 0;
    return runner.peer_known ? 0 : 5;
}
