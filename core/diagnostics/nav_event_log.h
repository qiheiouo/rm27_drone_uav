/* Fixed-capacity diagnostic event log for MCU and host replay. */
#ifndef NAV_EVENT_LOG_H
#define NAV_EVENT_LOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NAV_EVENT_LOG_CAPACITY 64u

typedef enum {
    NAV_LOG_RUNTIME_STARTED = 0,
    NAV_LOG_INPUT_STALE,
    NAV_LOG_INPUT_DUPLICATE,
    NAV_LOG_INPUT_OUT_OF_ORDER,
    NAV_LOG_INPUT_NONFINITE,
    NAV_LOG_INPUT_INVALID,
    NAV_LOG_WATCHDOG_OVERRUN,
    NAV_LOG_WATCHDOG_TRIPPED,
    NAV_LOG_MISSION_TRANSITION,
    NAV_LOG_IMPACT_CONFIRMED,
    NAV_LOG_RELOCALIZED,
    NAV_LOG_OBSTACLE_RISK,
    NAV_LOG_SWARM_CONFLICT,
    NAV_LOG_TRAJECTORY_DETOUR,
    NAV_LOG_TRAJECTORY_INVALID,
    NAV_LOG_SWARM_LINK_HOLD,
    NAV_LOG_SWARM_LINK_RECOVERED
} NavLogEventCode;

typedef struct {
    uint32_t sequence;
    uint32_t timestamp_ms;
    uint32_t data;
    float value;
    NavLogEventCode code;
} NavEventRecord;

typedef struct {
    NavEventRecord records[NAV_EVENT_LOG_CAPACITY];
    uint32_t next_sequence;
    uint32_t dropped;
    uint16_t start;
    uint16_t count;
} NavEventLog;

void nav_event_log_init(NavEventLog *log);
void nav_event_log_push(NavEventLog *log, uint32_t timestamp_ms,
                        NavLogEventCode code, uint32_t data, float value);
uint16_t nav_event_log_count(const NavEventLog *log);
uint8_t nav_event_log_get(const NavEventLog *log, uint16_t oldest_offset,
                          NavEventRecord *out);
const char *nav_log_event_code_name(NavLogEventCode code);

#ifdef __cplusplus
}
#endif

#endif /* NAV_EVENT_LOG_H */
