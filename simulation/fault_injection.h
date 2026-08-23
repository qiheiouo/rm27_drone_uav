/* Deterministic, fixed-memory fault injection for host-side regression. */
#ifndef FAULT_INJECTION_H
#define FAULT_INJECTION_H

#include <stdint.h>

#include "nav_runtime.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SIM_FAULT_MAX_RULES 8u

typedef enum {
    SIM_FAULT_SOURCE_IMU = 0,
    SIM_FAULT_SOURCE_FLOW,
    SIM_FAULT_SOURCE_ODOMETRY,
    SIM_FAULT_SOURCE_TARGET,
    SIM_FAULT_SOURCE_HOME,
    SIM_FAULT_SOURCE_SCHEDULER,
    SIM_FAULT_SOURCE_COUNT
} SimFaultSource;

typedef enum {
    SIM_FAULT_DROPOUT = 0,
    SIM_FAULT_FREEZE_TIMESTAMP,
    SIM_FAULT_DELAY_TIMESTAMP,
    SIM_FAULT_ROLLBACK_TIMESTAMP,
    SIM_FAULT_NONFINITE,
    SIM_FAULT_CYCLE_OVERRUN,
    SIM_FAULT_MODE_COUNT
} SimFaultMode;

typedef struct {
    SimFaultSource source;
    SimFaultMode mode;
    uint32_t start_ms;
    uint32_t end_ms;
    uint32_t magnitude_ms;
    uint16_t max_applications;
} SimFaultRule;

typedef struct {
    SimFaultRule rules[SIM_FAULT_MAX_RULES];
    uint8_t count;
} SimFaultPlan;

typedef struct {
    uint32_t active_rule_mask;
    uint32_t applied_rule_mask;
    uint32_t applied_source_mask;
    uint32_t applied_mode_mask;
    uint32_t scheduler_offset_ms;
    uint32_t frozen_timestamp_ms[SIM_FAULT_MAX_RULES];
    uint16_t application_count[SIM_FAULT_MAX_RULES];
    uint8_t frozen_timestamp_valid_mask;
} SimFaultState;

void sim_fault_plan_init(SimFaultPlan *plan);
uint8_t sim_fault_plan_add(SimFaultPlan *plan, SimFaultSource source,
                           SimFaultMode mode, uint32_t start_ms,
                           uint32_t end_ms, uint32_t magnitude_ms,
                           uint16_t max_applications);
void sim_fault_state_init(SimFaultState *state);

/*
 * Apply active rules after the simulator produced a normal sensor frame and
 * before NavRuntime consumes it. Scheduler overruns shift the runtime clock
 * and all available source timestamps together, modelling a delayed task
 * wake-up without manufacturing stale sensor data.
 */
void sim_fault_apply(const SimFaultPlan *plan, SimFaultState *state,
                     uint32_t simulation_time_ms, NavRuntimeInput *input,
                     ImuSample *imu, FlowFrame *flow, OdomSample *odometry,
                     PixelObs *target, PixelObs *home);

const char *sim_fault_source_name(SimFaultSource source);
const char *sim_fault_mode_name(SimFaultMode mode);

#ifdef __cplusplus
}
#endif

#endif /* FAULT_INJECTION_H */
