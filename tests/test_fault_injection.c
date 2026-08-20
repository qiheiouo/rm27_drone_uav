#include <stdio.h>
#include <string.h>

#include "fault_injection.h"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; \
} } while (0)

static void make_frame(uint32_t timestamp_ms, NavRuntimeInput *input,
                       ImuSample *imu, FlowFrame *flow,
                       OdomSample *odometry, PixelObs *target,
                       PixelObs *home)
{
    memset(input, 0, sizeof(*input));
    memset(imu, 0, sizeof(*imu));
    memset(flow, 0, sizeof(*flow));
    memset(odometry, 0, sizeof(*odometry));
    memset(target, 0, sizeof(*target));
    memset(home, 0, sizeof(*home));
    input->timestamp_ms = timestamp_ms;
    input->imu = imu;
    input->flow = flow;
    input->odometry = odometry;
    input->target_pixel = target;
    input->home_pixel = home;
    input->dt = 0.01f;
    imu->timestamp_ms = timestamp_ms;
    flow->timestamp_ms = timestamp_ms;
    odometry->timestamp_ms = timestamp_ms;
    target->timestamp_ms = timestamp_ms;
    home->timestamp_ms = timestamp_ms;
    flow->count = 1u;
    target->visible = 1u;
    target->size_px = 10.0f;
    home->visible = 1u;
    home->size_px = 10.0f;
}

int main(void)
{
    SimFaultPlan plan;
    SimFaultState state;
    NavRuntimeInput input;
    ImuSample imu;
    FlowFrame flow;
    OdomSample odometry;
    PixelObs target;
    PixelObs home;
    uint8_t index;

    sim_fault_plan_init(&plan);
    CHECK(sim_fault_plan_add(&plan, SIM_FAULT_SOURCE_SCHEDULER,
                             SIM_FAULT_DROPOUT, 0u, 10u, 0u, 0u) == 0u);
    CHECK(sim_fault_plan_add(&plan, SIM_FAULT_SOURCE_IMU,
                             SIM_FAULT_CYCLE_OVERRUN,
                             0u, 10u, 20u, 0u) == 0u);
    CHECK(sim_fault_plan_add(&plan, SIM_FAULT_SOURCE_IMU,
                             SIM_FAULT_DELAY_TIMESTAMP,
                             20u, 10u, 20u, 0u) == 0u);
    for (index = 0u; index < SIM_FAULT_MAX_RULES; index++) {
        CHECK(sim_fault_plan_add(&plan, SIM_FAULT_SOURCE_TARGET,
                                 SIM_FAULT_DROPOUT,
                                 index, index, 0u, 0u) == 1u);
    }
    CHECK(sim_fault_plan_add(&plan, SIM_FAULT_SOURCE_TARGET,
                             SIM_FAULT_DROPOUT, 20u, 20u, 0u, 0u) == 0u);

    sim_fault_plan_init(&plan);
    sim_fault_state_init(&state);
    CHECK(sim_fault_plan_add(&plan, SIM_FAULT_SOURCE_SCHEDULER,
                             SIM_FAULT_CYCLE_OVERRUN,
                             100u, 200u, 60u, 2u) == 1u);
    make_frame(100u, &input, &imu, &flow, &odometry, &target, &home);
    sim_fault_apply(&plan, &state, 100u, &input, &imu, &flow, &odometry,
                    &target, &home);
    CHECK(input.timestamp_ms == 160u);
    CHECK(imu.timestamp_ms == 160u);
    CHECK(target.timestamp_ms == 160u);
    CHECK(state.scheduler_offset_ms == 60u);
    make_frame(110u, &input, &imu, &flow, &odometry, &target, &home);
    sim_fault_apply(&plan, &state, 110u, &input, &imu, &flow, &odometry,
                    &target, &home);
    CHECK(input.timestamp_ms == 230u);
    CHECK(state.scheduler_offset_ms == 120u);
    make_frame(120u, &input, &imu, &flow, &odometry, &target, &home);
    sim_fault_apply(&plan, &state, 120u, &input, &imu, &flow, &odometry,
                    &target, &home);
    CHECK(input.timestamp_ms == 240u);
    CHECK(state.application_count[0] == 2u);

    sim_fault_plan_init(&plan);
    sim_fault_state_init(&state);
    CHECK(sim_fault_plan_add(&plan, SIM_FAULT_SOURCE_TARGET,
                             SIM_FAULT_FREEZE_TIMESTAMP,
                             100u, 200u, 0u, 0u) == 1u);
    make_frame(100u, &input, &imu, &flow, &odometry, &target, &home);
    sim_fault_apply(&plan, &state, 100u, &input, &imu, &flow, &odometry,
                    &target, &home);
    CHECK(target.timestamp_ms == 100u);
    make_frame(110u, &input, &imu, &flow, &odometry, &target, &home);
    sim_fault_apply(&plan, &state, 110u, &input, &imu, &flow, &odometry,
                    &target, &home);
    CHECK(target.timestamp_ms == 100u);
    CHECK(state.application_count[0] == 2u);

    sim_fault_plan_init(&plan);
    sim_fault_state_init(&state);
    CHECK(sim_fault_plan_add(&plan, SIM_FAULT_SOURCE_FLOW,
                             SIM_FAULT_DROPOUT,
                             100u, 100u, 0u, 0u) == 1u);
    make_frame(100u, &input, &imu, &flow, &odometry, &target, &home);
    sim_fault_apply(&plan, &state, 100u, &input, &imu, &flow, &odometry,
                    &target, &home);
    CHECK(input.flow == 0);
    CHECK(input.imu != 0);

    sim_fault_plan_init(&plan);
    sim_fault_state_init(&state);
    CHECK(sim_fault_plan_add(&plan, SIM_FAULT_SOURCE_HOME,
                             SIM_FAULT_DELAY_TIMESTAMP,
                             100u, 100u, 50u, 0u) == 1u);
    make_frame(100u, &input, &imu, &flow, &odometry, &target, &home);
    sim_fault_apply(&plan, &state, 100u, &input, &imu, &flow, &odometry,
                    &target, &home);
    CHECK(home.timestamp_ms == 50u);

    sim_fault_plan_init(&plan);
    sim_fault_state_init(&state);
    CHECK(sim_fault_plan_add(&plan, SIM_FAULT_SOURCE_IMU,
                             SIM_FAULT_ROLLBACK_TIMESTAMP,
                             100u, 100u, 40u, 1u) == 1u);
    make_frame(100u, &input, &imu, &flow, &odometry, &target, &home);
    sim_fault_apply(&plan, &state, 100u, &input, &imu, &flow, &odometry,
                    &target, &home);
    CHECK(imu.timestamp_ms == 60u);
    CHECK(state.application_count[0] == 1u);

    sim_fault_plan_init(&plan);
    sim_fault_state_init(&state);
    CHECK(sim_fault_plan_add(&plan, SIM_FAULT_SOURCE_TARGET,
                             SIM_FAULT_NONFINITE,
                             100u, 100u, 0u, 0u) == 1u);
    make_frame(100u, &input, &imu, &flow, &odometry, &target, &home);
    sim_fault_apply(&plan, &state, 100u, &input, &imu, &flow, &odometry,
                    &target, &home);
    CHECK(!nav_isfinite(target.size_px));
    CHECK(state.applied_rule_mask == 1u);
    CHECK(strcmp(sim_fault_source_name(SIM_FAULT_SOURCE_TARGET), "target") == 0);
    CHECK(strcmp(sim_fault_mode_name(SIM_FAULT_NONFINITE), "nonfinite") == 0);

    if (failures == 0) {
        printf("fault injection tests passed\n");
    }
    return failures == 0 ? 0 : 1;
}
