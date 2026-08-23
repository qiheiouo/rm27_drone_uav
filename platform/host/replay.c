/* Deterministic CSV replay for recorded navigation inputs. */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "nav_runtime.h"

#define REPLAY_FIELD_COUNT 31u
#define REPLAY_LINE_CAPACITY 1024u

typedef struct {
    uint32_t timestamp_ms;
    ImuSample imu;
    float tof_height;
    OdomSample odometry;
    PixelObs target;
    PixelObs home;
    uint8_t start_command;
    uint8_t request_return;
    uint8_t request_emergency;
    uint8_t dock_contact;
    uint8_t charging_detected;
    uint8_t wireless_charge_ready;
} ReplaySample;

typedef struct {
    uint32_t digest;
    uint32_t sample_count;
    uint32_t event_count;
    uint32_t dropped_events;
} ReplayResult;

static uint32_t hash_u32(uint32_t hash, uint32_t value)
{
    uint8_t index;
    for (index = 0u; index < 4u; index++) {
        hash ^= (value >> (8u * index)) & 0xffu;
        hash *= 16777619u;
    }
    return hash;
}

static uint32_t quantize_float(float value)
{
    float scaled = value * 10000.0f;
    if (!nav_isfinite(scaled)) return 0x7fffffffu;
    if (scaled > 2147483647.0f) return 0x7fffffffu;
    if (scaled < -2147483647.0f) return 0x80000001u;
    int32_t quantized = (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
    return (uint32_t)quantized;
}

static uint8_t parse_values(char *line, double *values)
{
    char *cursor = line;
    uint8_t index;
    for (index = 0u; index < REPLAY_FIELD_COUNT; index++) {
        char *end;
        values[index] = strtod(cursor, &end);
        if (end == cursor) return 0u;
        cursor = end;
        while (*cursor == ' ' || *cursor == '\t') cursor++;
        if (index + 1u < REPLAY_FIELD_COUNT) {
            if (*cursor != ',') return 0u;
            cursor++;
        }
    }
    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\r' ||
           *cursor == '\n') cursor++;
    return (*cursor == '\0') ? 1u : 0u;
}

static uint8_t replay_sample_parse(char *line, ReplaySample *sample)
{
    double value[REPLAY_FIELD_COUNT];
    uint8_t index;
    if (!parse_values(line, value)) return 0u;
    for (index = 0u; index < REPLAY_FIELD_COUNT; index++) {
        if (!isfinite(value[index])) return 0u;
    }
    if (value[0] < 0.0 || value[0] > 4294967295.0) return 0u;

    sample->timestamp_ms = (uint32_t)value[0];
    sample->imu.timestamp_ms = sample->timestamp_ms;
    sample->imu.accel = vec3((float)value[1], (float)value[2], (float)value[3]);
    sample->imu.gyro = vec3((float)value[4], (float)value[5], (float)value[6]);
    sample->tof_height = (float)value[7];

    sample->odometry.valid = value[8] != 0.0 ? 1u : 0u;
    sample->odometry.timestamp_ms = sample->timestamp_ms;
    sample->odometry.pos = vec3((float)value[9], (float)value[10],
                                (float)value[11]);
    sample->odometry.vel = vec3((float)value[12], (float)value[13],
                                (float)value[14]);
    sample->odometry.yaw = (float)value[15];
    sample->odometry.yaw_rate = (float)value[16];
    sample->odometry.att = quat_from_axis_angle(vec3(0.0f, 0.0f, 1.0f),
                                                sample->odometry.yaw);

    sample->target.visible = value[17] != 0.0 ? 1u : 0u;
    sample->target.u = (float)value[18];
    sample->target.v = (float)value[19];
    sample->target.size_px = (float)value[20];
    sample->target.timestamp_ms = sample->timestamp_ms;
    sample->home.visible = value[21] != 0.0 ? 1u : 0u;
    sample->home.u = (float)value[22];
    sample->home.v = (float)value[23];
    sample->home.size_px = (float)value[24];
    sample->home.timestamp_ms = sample->timestamp_ms;

    sample->start_command = value[25] != 0.0 ? 1u : 0u;
    sample->request_return = value[26] != 0.0 ? 1u : 0u;
    sample->request_emergency = value[27] != 0.0 ? 1u : 0u;
    sample->dock_contact = value[28] != 0.0 ? 1u : 0u;
    sample->charging_detected = value[29] != 0.0 ? 1u : 0u;
    sample->wireless_charge_ready = value[30] != 0.0 ? 1u : 0u;
    return 1u;
}

static uint32_t hash_output(uint32_t hash, const NavRuntimeOutput *output,
                            uint32_t timestamp_ms)
{
    hash = hash_u32(hash, timestamp_ms);
    hash = hash_u32(hash, (uint32_t)output->mission.state);
    hash = hash_u32(hash, (uint32_t)output->nav.status);
    hash = hash_u32(hash, output->event_flags);
    hash = hash_u32(hash, output->health.stale_source_mask);
    hash = hash_u32(hash, output->health.duplicate_source_mask);
    hash = hash_u32(hash, quantize_float(output->nav.pos.x));
    hash = hash_u32(hash, quantize_float(output->nav.pos.y));
    hash = hash_u32(hash, quantize_float(output->nav.pos.z));
    hash = hash_u32(hash, quantize_float(output->control.accel_cmd.x));
    hash = hash_u32(hash, quantize_float(output->control.accel_cmd.y));
    hash = hash_u32(hash, quantize_float(output->control.accel_cmd.z));
    hash = hash_u32(hash, quantize_float(output->control.yaw_rate_cmd));
    return hash;
}

static void print_events(const NavRuntime *runtime)
{
    const NavEventLog *log = nav_runtime_event_log(runtime);
    uint16_t index;
    NavEventRecord record;
    for (index = 0u; index < nav_event_log_count(log); index++) {
        if (nav_event_log_get(log, index, &record)) {
            printf("# event,%u,%u,%s,%u,%.4f\n",
                   (unsigned int)record.sequence,
                   (unsigned int)record.timestamp_ms,
                   nav_log_event_code_name(record.code),
                   (unsigned int)record.data, record.value);
        }
    }
}

static int replay_once(const char *path, uint8_t emit, ReplayResult *result)
{
    FILE *input_file;
    char line[REPLAY_LINE_CAPACITY];
    uint32_t line_number = 0u;
    uint32_t previous_timestamp_ms = 0u;
    uint8_t have_previous = 0u;
    NavRuntimeConfig config;
    NavRuntime runtime;

    input_file = fopen(path, "r");
    if (input_file == 0) {
        fprintf(stderr, "cannot open replay input: %s\n", path);
        return 0;
    }
    nav_runtime_config_default(&config);
    config.estimator.mode = EST_MODE_TRUTH;
    if (nav_runtime_init(&runtime, &config) != NAV_CONFIG_ERROR_NONE) {
        fclose(input_file);
        fprintf(stderr, "replay configuration invalid\n");
        return 0;
    }

    result->digest = 2166136261u;
    result->sample_count = 0u;
    if (emit) {
        printf("timestamp_ms,mission_state,estimator_status,event_flags,"
               "health_stale,health_duplicate,pos_x,pos_y,pos_z,"
               "accel_x,accel_y,accel_z,yaw_rate\n");
    }

    while (fgets(line, sizeof(line), input_file) != 0) {
        ReplaySample sample;
        NavRuntimeInput input;
        float dt = config.nominal_dt_s;
        char *cursor = line;
        line_number++;
        while (*cursor == ' ' || *cursor == '\t') cursor++;
        if (*cursor == '\0' || *cursor == '\r' || *cursor == '\n' ||
            *cursor == '#') {
            continue;
        }
        if (!replay_sample_parse(cursor, &sample)) {
            fclose(input_file);
            fprintf(stderr, "invalid replay row at line %u\n",
                    (unsigned int)line_number);
            return 0;
        }
        if (have_previous) {
            uint32_t elapsed_ms = sample.timestamp_ms - previous_timestamp_ms;
            dt = (float)elapsed_ms * 0.001f;
            if (dt > 0.2f) dt = 0.2f;
        }
        previous_timestamp_ms = sample.timestamp_ms;
        have_previous = 1u;

        input.timestamp_ms = sample.timestamp_ms;
        input.imu = &sample.imu;
        input.flow = 0;
        input.odometry = &sample.odometry;
        input.tof_height = sample.tof_height;
        input.target_pixel = &sample.target;
        input.home_pixel = &sample.home;
        input.obstacles = 0;
        input.swarm = 0;
        input.start_command = sample.start_command;
        input.request_return = sample.request_return;
        input.request_emergency = sample.request_emergency;
        input.dock_contact = sample.dock_contact;
        input.charging_detected = sample.charging_detected;
        input.wireless_charge_ready = sample.wireless_charge_ready;
        input.dt = dt;
        if (!nav_runtime_step(&runtime, &input)) {
            fclose(input_file);
            fprintf(stderr, "runtime rejected replay row at line %u\n",
                    (unsigned int)line_number);
            return 0;
        }

        result->digest = hash_output(result->digest, &runtime.output,
                                     sample.timestamp_ms);
        result->sample_count++;
        if (emit) {
            printf("%u,%s,%s,0x%08x,0x%08x,0x%08x,"
                   "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                   (unsigned int)sample.timestamp_ms,
                   mission_state_name(runtime.output.mission.state),
                   est_status_name(runtime.output.nav.status),
                   (unsigned int)runtime.output.event_flags,
                   (unsigned int)runtime.output.health.stale_source_mask,
                   (unsigned int)runtime.output.health.duplicate_source_mask,
                   runtime.output.nav.pos.x, runtime.output.nav.pos.y,
                   runtime.output.nav.pos.z,
                   runtime.output.control.accel_cmd.x,
                   runtime.output.control.accel_cmd.y,
                   runtime.output.control.accel_cmd.z,
                   runtime.output.control.yaw_rate_cmd);
        }
    }
    fclose(input_file);
    result->event_count = nav_event_log_count(&runtime.event_log);
    result->dropped_events = runtime.event_log.dropped;
    if (emit) print_events(&runtime);
    return result->sample_count > 0u ? 1 : 0;
}

static void print_usage(const char *program)
{
    printf("usage: %s [--verify] input.csv\n", program);
}

int main(int argc, char **argv)
{
    ReplayResult first;
    ReplayResult second;
    const char *path;
    uint8_t verify = 0u;

    if (argc == 3 && strcmp(argv[1], "--verify") == 0) {
        verify = 1u;
        path = argv[2];
    } else if (argc == 2) {
        path = argv[1];
    } else {
        print_usage(argv[0]);
        return 1;
    }

    if (!replay_once(path, verify ? 0u : 1u, &first)) return 2;
    if (verify) {
        if (!replay_once(path, 0u, &second)) return 2;
        if (first.digest != second.digest ||
            first.sample_count != second.sample_count ||
            first.event_count != second.event_count ||
            first.dropped_events != second.dropped_events) {
            printf("REPLAY_FAILED non-deterministic output\n");
            return 3;
        }
    }
    printf("REPLAY_SUCCESS samples=%u events=%u dropped=%u digest=%08x\n",
           (unsigned int)first.sample_count,
           (unsigned int)first.event_count,
           (unsigned int)first.dropped_events,
           (unsigned int)first.digest);
    return 0;
}
