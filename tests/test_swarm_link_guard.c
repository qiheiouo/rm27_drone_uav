#include <stdio.h>

#include "swarm_link_guard.h"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; \
} } while (0)

static CollisionReport no_conflict(void)
{
    CollisionReport report;
    report.conflict = 0u;
    report.other_agent_id = 0u;
    report.min_separation = 1e9f;
    report.time_to_conflict_s = 1e9f;
    report.closing_speed_mps = 0.0f;
    report.risk = SWARM_RISK_NONE;
    return report;
}

static CollisionReport conflict_with(uint8_t agent_id)
{
    CollisionReport report = no_conflict();
    report.conflict = 1u;
    report.other_agent_id = agent_id;
    report.min_separation = 0.3f;
    report.time_to_conflict_s = 1.0f;
    report.closing_speed_mps = 1.0f;
    report.risk = SWARM_RISK_WARNING;
    return report;
}

static void add_peer(SwarmView *view, uint8_t agent_id)
{
    swarm_view_init(view, 2u);
    view->other_count = 1u;
    view->others[0].agent_id = agent_id;
    view->others[0].valid = 1u;
    view->others[0].quality = 1.0f;
    view->others[0].age_s = 0.0f;
}

int main(void)
{
    SwarmLinkGuardConfig cfg;
    SwarmLinkGuard guard;
    SwarmLinkGuardOutput output;
    SwarmView view;
    NavState nav = {0};
    CollisionReport clear = no_conflict();
    CollisionReport conflict = conflict_with(1u);
    Vec3f first_hold;
    int index;

    nav.pos = vec3(1.0f, 2.0f, 1.2f);
    add_peer(&view, 1u);
    swarm_link_guard_default_config(&cfg);
    swarm_link_guard_init(&guard, &cfg);
    swarm_link_guard_update(&guard, &view, &conflict, &nav, 0.1f, &output);
    CHECK(output.state == SWARM_LINK_GUARD_CLEAR);
    CHECK(output.active == 0u);

    cfg.enabled = 1u;
    swarm_link_guard_init(&guard, &cfg);
    swarm_link_guard_update(&guard, &view, &conflict, &nav, 0.1f, &output);
    CHECK(output.state == SWARM_LINK_GUARD_MONITORING);
    CHECK(output.hazard_agent_id == 1u);
    CHECK(output.active == 0u);

    /* A visible peer whose conflict clears naturally must not cause a hold. */
    for (index = 0; index < 4; index++) {
        swarm_link_guard_update(&guard, &view, &clear, &nav, 0.1f, &output);
    }
    CHECK(output.state == SWARM_LINK_GUARD_CLEAR);
    CHECK(output.active == 0u);

    /* A monitored peer disappearing captures one fixed hold position. */
    swarm_link_guard_update(&guard, &view, &conflict, &nav, 0.1f, &output);
    swarm_view_init(&view, 2u);
    swarm_link_guard_update(&guard, &view, &clear, &nav, 0.1f, &output);
    CHECK(output.state == SWARM_LINK_GUARD_HOLDING);
    CHECK(output.active == 1u);
    CHECK(output.peer_present == 0u);
    CHECK(vec3_dist(output.hold_position, nav.pos) < 1e-6f);
    first_hold = output.hold_position;

    /* Another visible conflict cannot replace the still-missing hazard. */
    add_peer(&view, 3u);
    {
        CollisionReport other_conflict = conflict_with(3u);
        swarm_link_guard_update(&guard, &view, &other_conflict, &nav,
                                0.1f, &output);
    }
    CHECK(output.state == SWARM_LINK_GUARD_HOLDING);
    CHECK(output.hazard_agent_id == 1u);
    CHECK(vec3_dist(output.hold_position, first_hold) < 1e-6f);

    /* Restoration remains held until a stable recovery interval passes. */
    add_peer(&view, 1u);
    swarm_link_guard_update(&guard, &view, &clear, &nav, 0.1f, &output);
    CHECK(output.state == SWARM_LINK_GUARD_RECOVERING);
    CHECK(output.active == 1u);
    for (index = 0; index < 4; index++) {
        swarm_link_guard_update(&guard, &view, &clear, &nav, 0.1f, &output);
    }
    CHECK(output.state == SWARM_LINK_GUARD_CLEAR);
    CHECK(output.active == 0u);

    /* A second dropout during recovery returns to the original hold point. */
    swarm_link_guard_update(&guard, &view, &conflict, &nav, 0.1f, &output);
    swarm_view_init(&view, 2u);
    swarm_link_guard_update(&guard, &view, &clear, &nav, 0.1f, &output);
    first_hold = output.hold_position;
    add_peer(&view, 1u);
    swarm_link_guard_update(&guard, &view, &clear, &nav, 0.1f, &output);
    nav.pos = vec3(5.0f, 5.0f, 1.2f);
    swarm_view_init(&view, 2u);
    swarm_link_guard_update(&guard, &view, &clear, &nav, 0.1f, &output);
    CHECK(output.state == SWARM_LINK_GUARD_HOLDING);
    CHECK(vec3_dist(output.hold_position, first_hold) < 1e-6f);

    if (failures == 0) {
        printf("test_swarm_link_guard: PASS\n");
        return 0;
    }
    printf("test_swarm_link_guard: %d FAILURES\n", failures);
    return 1;
}
