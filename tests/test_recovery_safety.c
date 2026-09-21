#include <stdio.h>
#include "scenario.h"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; \
} } while (0)

static SafetyDecision update_safety(SafetyMonitor *monitor, NavState *nav,
                                    CollisionRiskLevel risk,
                                    uint8_t swarm_link_guard_active,
                                    float dt)
{
    SafetyInput input;
    input.nav = nav;
    input.home_position = vec3_zero();
    input.impact_state = NO_IMPACT;
    input.collision_risk = risk;
    input.trajectory_valid = 1u;
    input.controller_saturated = 0u;
    input.swarm_link_guard_active = swarm_link_guard_active;
    input.dt = dt;
    return safety_update_full(monitor, &input);
}

int main(void)
{
    Scenario scenario;
    ImpactRecovery recovery;
    ImpactRecoveryOutput output;
    NavState nav = {0};
    SafetyMonitor safety;
    SafetyDecision decision;
    int index;

    scenario_default(&scenario);
    nav.att = quat_identity();
    nav.status = EST_TRACKING;
    nav.quality = 1.0f;
    nav.validity = NAV_VALID_VALID;
    nav.pos = vec3(1.0f, 2.0f, 1.0f);
    nav.vel = vec3_zero();

    impact_recovery_init(&recovery, &scenario.recovery);
    impact_recovery_start(&recovery, &nav);
    for (index = 0; index < 100 && recovery.stage != RECOVERY_BREAKAWAY; index++) {
        impact_recovery_update(&recovery, &nav, 1u, 0.01f, &output);
    }
    CHECK(recovery.stage == RECOVERY_BREAKAWAY);
    nav.pos = recovery.breakaway_target;
    impact_recovery_update(&recovery, &nav, 1u, 0.01f, &output);
    CHECK(recovery.stage == RECOVERY_COMPLETE);
    CHECK(output.complete == 1u);

    scenario.safety.collision_critical_timeout_s = 0.08f;
    safety_init(&safety, &scenario.safety);
    for (index = 0; index < 4; index++) {
        decision = update_safety(&safety, &nav, COLLISION_RISK_CRITICAL, 0u, 0.01f);
    }
    CHECK(decision.request_emergency == 0u);
    for (index = 0; index < 6; index++) {
        decision = update_safety(&safety, &nav, COLLISION_RISK_CRITICAL, 0u, 0.01f);
    }
    CHECK(decision.request_emergency == 1u);
    CHECK((decision.reason_mask & SAFETY_REASON_COLLISION) != 0u);

    scenario.safety.soft_return_deadline_s = 0.03f;
    scenario.safety.hard_return_deadline_s = 0.06f;
    scenario.safety.max_mission_time_s = 0.06f;
    safety_init(&safety, &scenario.safety);
    for (index = 0; index < 4; index++) {
        decision = update_safety(&safety, &nav, COLLISION_RISK_NONE, 0u, 0.01f);
    }
    CHECK(decision.request_return == 1u);
    CHECK(decision.request_emergency == 0u);
    for (index = 0; index < 3; index++) {
        decision = update_safety(&safety, &nav, COLLISION_RISK_NONE, 0u, 0.01f);
    }
    CHECK(decision.request_emergency == 1u);
    CHECK((decision.reason_mask & SAFETY_REASON_HARD_DEADLINE) != 0u);

    scenario_default(&scenario);
    safety_init(&safety, &scenario.safety);
    decision = update_safety(&safety, &nav, COLLISION_RISK_NONE, 1u, 0.01f);
    CHECK(decision.level == SAFETY_DEGRADED);
    CHECK((decision.reason_mask & SAFETY_REASON_SWARM_LINK) != 0u);
    CHECK(decision.request_return == 0u);
    CHECK(decision.request_emergency == 0u);

    if (failures == 0) {
        printf("test_recovery_safety: PASS\n");
        return 0;
    }
    printf("test_recovery_safety: %d FAILURES\n", failures);
    return 1;
}
