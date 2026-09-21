#include "nav_event_log.h"

void nav_event_log_init(NavEventLog *log)
{
    log->next_sequence = 0u;
    log->dropped = 0u;
    log->start = 0u;
    log->count = 0u;
}

void nav_event_log_push(NavEventLog *log, uint32_t timestamp_ms,
                        NavLogEventCode code, uint32_t data, float value)
{
    uint16_t index;
    if (log->count < NAV_EVENT_LOG_CAPACITY) {
        index = (uint16_t)((log->start + log->count) % NAV_EVENT_LOG_CAPACITY);
        log->count++;
    } else {
        index = log->start;
        log->start = (uint16_t)((log->start + 1u) % NAV_EVENT_LOG_CAPACITY);
        log->dropped++;
    }
    log->records[index].sequence = log->next_sequence++;
    log->records[index].timestamp_ms = timestamp_ms;
    log->records[index].code = code;
    log->records[index].data = data;
    log->records[index].value = value;
}

uint16_t nav_event_log_count(const NavEventLog *log)
{
    return log->count;
}

uint8_t nav_event_log_get(const NavEventLog *log, uint16_t oldest_offset,
                          NavEventRecord *out)
{
    uint16_t index;
    if (oldest_offset >= log->count || out == 0) return 0u;
    index = (uint16_t)((log->start + oldest_offset) % NAV_EVENT_LOG_CAPACITY);
    *out = log->records[index];
    return 1u;
}

const char *nav_log_event_code_name(NavLogEventCode code)
{
    switch (code) {
    case NAV_LOG_RUNTIME_STARTED: return "runtime-started";
    case NAV_LOG_INPUT_STALE: return "input-stale";
    case NAV_LOG_INPUT_DUPLICATE: return "input-duplicate";
    case NAV_LOG_INPUT_OUT_OF_ORDER: return "input-out-of-order";
    case NAV_LOG_INPUT_NONFINITE: return "input-nonfinite";
    case NAV_LOG_INPUT_INVALID: return "input-invalid";
    case NAV_LOG_WATCHDOG_OVERRUN: return "watchdog-overrun";
    case NAV_LOG_WATCHDOG_TRIPPED: return "watchdog-tripped";
    case NAV_LOG_MISSION_TRANSITION: return "mission-transition";
    case NAV_LOG_IMPACT_CONFIRMED: return "impact-confirmed";
    case NAV_LOG_RELOCALIZED: return "relocalized";
    case NAV_LOG_OBSTACLE_RISK: return "obstacle-risk";
    case NAV_LOG_SWARM_CONFLICT: return "swarm-conflict";
    case NAV_LOG_SWARM_LINK_HOLD: return "swarm-link-hold";
    case NAV_LOG_SWARM_LINK_RECOVERED: return "swarm-link-recovered";
    case NAV_LOG_TRAJECTORY_DETOUR: return "trajectory-detour";
    case NAV_LOG_TRAJECTORY_INVALID: return "trajectory-invalid";
    default: return "unknown";
    }
}
