#include <math.h>
#include <string.h>

#include "fault_injection.h"

static uint8_t rule_valid(SimFaultSource source, SimFaultMode mode,
                          uint32_t start_ms, uint32_t end_ms,
                          uint32_t magnitude_ms)
{
    if (source >= SIM_FAULT_SOURCE_COUNT || mode >= SIM_FAULT_MODE_COUNT ||
        end_ms < start_ms) {
        return 0u;
    }
    if ((source == SIM_FAULT_SOURCE_SCHEDULER) !=
        (mode == SIM_FAULT_CYCLE_OVERRUN)) {
        return 0u;
    }
    if ((mode == SIM_FAULT_DELAY_TIMESTAMP ||
         mode == SIM_FAULT_ROLLBACK_TIMESTAMP ||
         mode == SIM_FAULT_CYCLE_OVERRUN) && magnitude_ms == 0u) {
        return 0u;
    }
    return 1u;
}

void sim_fault_plan_init(SimFaultPlan *plan)
{
    if (plan != 0) {
        memset(plan, 0, sizeof(*plan));
    }
}

uint8_t sim_fault_plan_add(SimFaultPlan *plan, SimFaultSource source,
                           SimFaultMode mode, uint32_t start_ms,
                           uint32_t end_ms, uint32_t magnitude_ms,
                           uint16_t max_applications)
{
    SimFaultRule *rule;
    if (plan == 0 || plan->count >= SIM_FAULT_MAX_RULES ||
        !rule_valid(source, mode, start_ms, end_ms, magnitude_ms)) {
        return 0u;
    }
    rule = &plan->rules[plan->count++];
    rule->source = source;
    rule->mode = mode;
    rule->start_ms = start_ms;
    rule->end_ms = end_ms;
    rule->magnitude_ms = magnitude_ms;
    rule->max_applications = max_applications;
    return 1u;
}

void sim_fault_state_init(SimFaultState *state)
{
    if (state != 0) {
        memset(state, 0, sizeof(*state));
    }
}

static uint8_t rule_active(const SimFaultRule *rule,
                           const SimFaultState *state, uint8_t index,
                           uint32_t time_ms)
{
    if (time_ms < rule->start_ms || time_ms > rule->end_ms) {
        return 0u;
    }
    if (rule->max_applications != 0u &&
        state->application_count[index] >= rule->max_applications) {
        return 0u;
    }
    return 1u;
}

static uint32_t *sample_timestamp(SimFaultSource source,
                                  NavRuntimeInput *input,
                                  ImuSample *imu, FlowFrame *flow,
                                  OdomSample *odometry, PixelObs *target,
                                  PixelObs *home)
{
    switch (source) {
    case SIM_FAULT_SOURCE_IMU:
        return input->imu != 0 ? &imu->timestamp_ms : 0;
    case SIM_FAULT_SOURCE_FLOW:
        return input->flow != 0 ? &flow->timestamp_ms : 0;
    case SIM_FAULT_SOURCE_ODOMETRY:
        return input->odometry != 0 ? &odometry->timestamp_ms : 0;
    case SIM_FAULT_SOURCE_TARGET:
        return input->target_pixel != 0 ? &target->timestamp_ms : 0;
    case SIM_FAULT_SOURCE_HOME:
        return input->home_pixel != 0 ? &home->timestamp_ms : 0;
    default:
        return 0;
    }
}

static uint8_t apply_dropout(SimFaultSource source, NavRuntimeInput *input)
{
    switch (source) {
    case SIM_FAULT_SOURCE_IMU:
        if (input->imu == 0) return 0u;
        input->imu = 0;
        return 1u;
    case SIM_FAULT_SOURCE_FLOW:
        if (input->flow == 0) return 0u;
        input->flow = 0;
        return 1u;
    case SIM_FAULT_SOURCE_ODOMETRY:
        if (input->odometry == 0) return 0u;
        input->odometry = 0;
        return 1u;
    case SIM_FAULT_SOURCE_TARGET:
        if (input->target_pixel == 0) return 0u;
        input->target_pixel = 0;
        return 1u;
    case SIM_FAULT_SOURCE_HOME:
        if (input->home_pixel == 0) return 0u;
        input->home_pixel = 0;
        return 1u;
    default:
        return 0u;
    }
}

static uint8_t apply_nonfinite(SimFaultSource source,
                               const NavRuntimeInput *input,
                               ImuSample *imu, FlowFrame *flow,
                               OdomSample *odometry, PixelObs *target,
                               PixelObs *home)
{
    switch (source) {
    case SIM_FAULT_SOURCE_IMU:
        if (input->imu == 0) return 0u;
        imu->accel.x = NAN;
        return 1u;
    case SIM_FAULT_SOURCE_FLOW:
        if (input->flow == 0) return 0u;
        if (flow->count == 0u) flow->count = 1u;
        flow->feats[0].u = NAN;
        return 1u;
    case SIM_FAULT_SOURCE_ODOMETRY:
        if (input->odometry == 0) return 0u;
        odometry->valid = 1u;
        odometry->pos.x = NAN;
        return 1u;
    case SIM_FAULT_SOURCE_TARGET:
        if (input->target_pixel == 0) return 0u;
        target->visible = 1u;
        target->size_px = NAN;
        return 1u;
    case SIM_FAULT_SOURCE_HOME:
        if (input->home_pixel == 0) return 0u;
        home->visible = 1u;
        home->size_px = NAN;
        return 1u;
    default:
        return 0u;
    }
}

static void shift_available_timestamps(NavRuntimeInput *input,
                                       uint32_t offset_ms,
                                       ImuSample *imu, FlowFrame *flow,
                                       OdomSample *odometry,
                                       PixelObs *target, PixelObs *home)
{
    input->timestamp_ms += offset_ms;
    if (input->imu != 0) imu->timestamp_ms += offset_ms;
    if (input->flow != 0) flow->timestamp_ms += offset_ms;
    if (input->odometry != 0) odometry->timestamp_ms += offset_ms;
    if (input->target_pixel != 0) target->timestamp_ms += offset_ms;
    if (input->home_pixel != 0) home->timestamp_ms += offset_ms;
}

static void mark_applied(SimFaultState *state, uint8_t index,
                         const SimFaultRule *rule)
{
    state->active_rule_mask |= 1u << index;
    state->applied_rule_mask |= 1u << index;
    state->applied_source_mask |= 1u << (uint8_t)rule->source;
    state->applied_mode_mask |= 1u << (uint8_t)rule->mode;
    if (state->application_count[index] < 65535u) {
        state->application_count[index]++;
    }
}

void sim_fault_apply(const SimFaultPlan *plan, SimFaultState *state,
                     uint32_t simulation_time_ms, NavRuntimeInput *input,
                     ImuSample *imu, FlowFrame *flow, OdomSample *odometry,
                     PixelObs *target, PixelObs *home)
{
    uint8_t index;
    if (plan == 0 || state == 0 || input == 0 || imu == 0 || flow == 0 ||
        odometry == 0 || target == 0 || home == 0) {
        return;
    }
    state->active_rule_mask = 0u;

    /* Scheduler rules run first so all samples share the delayed clock. */
    for (index = 0u; index < plan->count; index++) {
        const SimFaultRule *rule = &plan->rules[index];
        if (rule->source == SIM_FAULT_SOURCE_SCHEDULER &&
            rule_active(rule, state, index, simulation_time_ms)) {
            state->scheduler_offset_ms += rule->magnitude_ms;
            mark_applied(state, index, rule);
        }
    }
    if (state->scheduler_offset_ms != 0u) {
        shift_available_timestamps(input, state->scheduler_offset_ms,
                                   imu, flow, odometry, target, home);
    }

    for (index = 0u; index < plan->count; index++) {
        const SimFaultRule *rule = &plan->rules[index];
        uint32_t *timestamp;
        uint8_t applied = 0u;
        if (rule->source == SIM_FAULT_SOURCE_SCHEDULER ||
            !rule_active(rule, state, index, simulation_time_ms)) {
            continue;
        }
        if (rule->mode == SIM_FAULT_DROPOUT) {
            applied = apply_dropout(rule->source, input);
        } else if (rule->mode == SIM_FAULT_NONFINITE) {
            applied = apply_nonfinite(rule->source, input, imu, flow,
                                      odometry, target, home);
        } else {
            timestamp = sample_timestamp(rule->source, input, imu, flow,
                                         odometry, target, home);
            if (timestamp != 0) {
                if (rule->mode == SIM_FAULT_FREEZE_TIMESTAMP) {
                    if ((state->frozen_timestamp_valid_mask &
                         (uint8_t)(1u << index)) == 0u) {
                        state->frozen_timestamp_ms[index] = *timestamp;
                        state->frozen_timestamp_valid_mask |=
                            (uint8_t)(1u << index);
                    }
                    *timestamp = state->frozen_timestamp_ms[index];
                    applied = 1u;
                } else if (rule->mode == SIM_FAULT_DELAY_TIMESTAMP ||
                           rule->mode == SIM_FAULT_ROLLBACK_TIMESTAMP) {
                    *timestamp -= rule->magnitude_ms;
                    applied = 1u;
                }
            }
        }
        if (applied) {
            mark_applied(state, index, rule);
        }
    }
}

const char *sim_fault_source_name(SimFaultSource source)
{
    switch (source) {
    case SIM_FAULT_SOURCE_IMU: return "imu";
    case SIM_FAULT_SOURCE_FLOW: return "flow";
    case SIM_FAULT_SOURCE_ODOMETRY: return "odometry";
    case SIM_FAULT_SOURCE_TARGET: return "target";
    case SIM_FAULT_SOURCE_HOME: return "home";
    case SIM_FAULT_SOURCE_SCHEDULER: return "scheduler";
    default: return "unknown";
    }
}

const char *sim_fault_mode_name(SimFaultMode mode)
{
    switch (mode) {
    case SIM_FAULT_DROPOUT: return "dropout";
    case SIM_FAULT_FREEZE_TIMESTAMP: return "freeze-timestamp";
    case SIM_FAULT_DELAY_TIMESTAMP: return "delay-timestamp";
    case SIM_FAULT_ROLLBACK_TIMESTAMP: return "rollback-timestamp";
    case SIM_FAULT_NONFINITE: return "nonfinite";
    case SIM_FAULT_CYCLE_OVERRUN: return "cycle-overrun";
    default: return "unknown";
    }
}
