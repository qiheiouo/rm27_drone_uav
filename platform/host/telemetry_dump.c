/* Validate and optionally convert binary navigation telemetry to CSV. */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "nav_telemetry.h"

#define TELEMETRY_COUNTER_HALF_RANGE 0x80000000u

static void print_usage(const char *program)
{
    printf("usage: %s [--csv] telemetry.bin\n", program);
}

static void print_csv_header(void)
{
    printf("sequence,timestamp_ms,mission_state,estimator_status,safety_level,"
           "flags,event_flags,safety_reason,health_stale,health_duplicate,"
           "health_out_of_order,health_nonfinite,health_invalid,cycle_gap_ms,"
           "pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,yaw,quality,"
           "target_x,target_y,target_z,home_x,home_y,home_z,"
           "guidance_vx,guidance_vy,guidance_vz,"
           "accel_x,accel_y,accel_z,yaw_rate,remaining_s\n");
}

static void print_csv_row(const NavTelemetrySnapshot *snapshot)
{
    printf("%u,%u,%s,%s,%s,0x%08x,0x%08x,0x%08x,"
           "0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,%u,"
           "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
           "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
           "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
           (unsigned int)snapshot->sequence,
           (unsigned int)snapshot->timestamp_ms,
           mission_state_name((MissionState)snapshot->mission_state),
           est_status_name((EstStatus)snapshot->estimator_status),
           safety_level_name((SafetyLevel)snapshot->safety_level),
           (unsigned int)snapshot->flags,
           (unsigned int)snapshot->event_flags,
           (unsigned int)snapshot->safety_reason_mask,
           (unsigned int)snapshot->stale_source_mask,
           (unsigned int)snapshot->duplicate_source_mask,
           (unsigned int)snapshot->out_of_order_source_mask,
           (unsigned int)snapshot->nonfinite_source_mask,
           (unsigned int)snapshot->invalid_source_mask,
           (unsigned int)snapshot->cycle_gap_ms,
           snapshot->position.x, snapshot->position.y, snapshot->position.z,
           snapshot->velocity.x, snapshot->velocity.y, snapshot->velocity.z,
           snapshot->yaw, snapshot->estimator_quality,
           snapshot->target_relative_position.x,
           snapshot->target_relative_position.y,
           snapshot->target_relative_position.z,
           snapshot->home_relative_position.x,
           snapshot->home_relative_position.y,
           snapshot->home_relative_position.z,
           snapshot->guidance_velocity.x,
           snapshot->guidance_velocity.y,
           snapshot->guidance_velocity.z,
           snapshot->acceleration_command.x,
           snapshot->acceleration_command.y,
           snapshot->acceleration_command.z,
           snapshot->yaw_rate_command, snapshot->remaining_mission_s);
}

int main(int argc, char **argv)
{
    const char *path;
    uint8_t emit_csv = 0u;
    FILE *file;
    uint8_t frame[NAV_TELEMETRY_FRAME_SIZE];
    NavTelemetrySnapshot snapshot;
    uint32_t frame_count = 0u;
    uint32_t dropped_frames = 0u;
    uint32_t elapsed_ms = 0u;
    uint32_t timestamp_anomalies = 0u;
    uint32_t previous_timestamp_ms = 0u;
    uint32_t previous_sequence = 0u;

    if (argc == 3 && strcmp(argv[1], "--csv") == 0) {
        emit_csv = 1u;
        path = argv[2];
    } else if (argc == 2) {
        path = argv[1];
    } else {
        print_usage(argv[0]);
        return 1;
    }

    file = fopen(path, "rb");
    if (file == 0) {
        fprintf(stderr, "cannot open telemetry input: %s\n", path);
        return 2;
    }
    if (emit_csv) print_csv_header();

    for (;;) {
        size_t count = fread(frame, 1u, NAV_TELEMETRY_FRAME_SIZE, file);
        NavTelemetryStatus status;
        if (count == 0u) {
            if (ferror(file)) {
                fprintf(stderr, "telemetry read failed\n");
                fclose(file);
                return 3;
            }
            break;
        }
        if (count != NAV_TELEMETRY_FRAME_SIZE) {
            fprintf(stderr, "truncated telemetry frame %u\n",
                    (unsigned int)frame_count);
            fclose(file);
            return 4;
        }
        status = nav_telemetry_decode(frame, NAV_TELEMETRY_FRAME_SIZE,
                                      &snapshot);
        if (status != NAV_TELEMETRY_OK) {
            fprintf(stderr, "invalid telemetry frame %u: %s\n",
                    (unsigned int)frame_count,
                    nav_telemetry_status_name(status));
            fclose(file);
            return 5;
        }
        if (frame_count != 0u) {
            uint32_t sequence_advance = snapshot.sequence - previous_sequence;
            uint32_t timestamp_advance =
                snapshot.timestamp_ms - previous_timestamp_ms;
            if (sequence_advance == 0u ||
                sequence_advance >= TELEMETRY_COUNTER_HALF_RANGE) {
                fprintf(stderr, "non-monotonic telemetry sequence at frame %u\n",
                        (unsigned int)frame_count);
                fclose(file);
                return 6;
            }
            dropped_frames += sequence_advance - 1u;
            if (timestamp_advance == 0u ||
                timestamp_advance >= TELEMETRY_COUNTER_HALF_RANGE) {
                timestamp_anomalies++;
            } else {
                elapsed_ms += timestamp_advance;
            }
        }
        previous_sequence = snapshot.sequence;
        previous_timestamp_ms = snapshot.timestamp_ms;
        frame_count++;
        if (emit_csv) print_csv_row(&snapshot);
    }
    if (fclose(file) != 0) {
        fprintf(stderr, "telemetry close failed\n");
        return 3;
    }
    if (frame_count == 0u) {
        fprintf(stderr, "telemetry file is empty\n");
        return 7;
    }
    {
        FILE *summary = emit_csv ? stderr : stdout;
        fprintf(summary,
                "TELEMETRY_OK frames=%u dropped=%u time_anomalies=%u "
                "duration_ms=%u schema=%u\n",
                (unsigned int)frame_count, (unsigned int)dropped_frames,
                (unsigned int)timestamp_anomalies, (unsigned int)elapsed_ms,
                (unsigned int)NAV_TELEMETRY_SCHEMA_VERSION);
    }
    return 0;
}
