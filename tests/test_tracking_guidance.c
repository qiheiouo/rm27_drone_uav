#include <stdio.h>
#include "scenario.h"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; \
} } while (0)

int main(void)
{
    TargetTrackerConfig tracker_cfg;
    TargetTracker tracker;
    TargetObs observation = {0};
    Scenario scenario;
    TerminalGuidance terminal;
    GuidanceOutput output;
    NavState nav = {0};
    int index;

    target_tracker_default_config(&tracker_cfg);
    target_tracker_init_config(&tracker, &tracker_cfg);
    observation.visible = 1u;
    observation.confidence = 0.9f;
    observation.rel_pos = vec3(3.0f, 0.3f, 0.1f);
    target_tracker_update(&tracker, &observation, 0.1f);
    CHECK(tracker.out.status == TARGET_TRACK_TRACKING);
    CHECK(tracker.out.confidence > 0.8f);

    observation.rel_pos.x = 2.8f;
    target_tracker_update(&tracker, &observation, 0.1f);
    CHECK(tracker.out.rel_vel.x < 0.0f);
    observation.visible = 0u;
    target_tracker_update(&tracker, &observation, 0.1f);
    CHECK(tracker.out.status == TARGET_TRACK_COASTING);
    for (index = 0; index < 12; index++) {
        target_tracker_update(&tracker, &observation, 0.1f);
    }
    CHECK(tracker.out.status == TARGET_TRACK_LOST);

    scenario_default(&scenario);
    terminal_guidance_init(&terminal, &scenario.terminal);
    nav.att = quat_identity();
    nav.status = EST_TRACKING;
    tracker.out.status = TARGET_TRACK_TRACKING;
    tracker.out.visible = 1u;
    tracker.out.confidence = 1.0f;
    tracker.out.rel_pos = vec3(3.0f, 0.0f, 0.0f);
    tracker.out.rel_vel = vec3_zero();
    tracker.out.bearing = vec3(1.0f, 0.0f, 0.0f);
    tracker.out.range = 3.0f;
    tracker.out.time_since_update = 0.0f;
    terminal_guidance_update(&terminal, &tracker.out, &nav, 0.01f, &output);
    CHECK(terminal.stage == TERMINAL_PURSUIT);
    CHECK(vec3_norm(output.accel_sp) <= scenario.terminal.max_accel_mps2 + 1e-4f);

    tracker.out.rel_pos = vec3(0.8f, 0.0f, 0.0f);
    tracker.out.range = 0.8f;
    terminal_guidance_update(&terminal, &tracker.out, &nav, 0.01f, &output);
    CHECK(terminal.stage == TERMINAL_FINAL_ALIGN);

    tracker.out.status = TARGET_TRACK_LOST;
    tracker.out.visible = 0u;
    tracker.out.confidence = 0.0f;
    terminal_guidance_update(&terminal, &tracker.out, &nav, 0.1f, &output);
    CHECK(terminal.stage == TERMINAL_REACQUIRE);
    for (index = 0; index < 12; index++) {
        terminal_guidance_update(&terminal, &tracker.out, &nav, 0.1f, &output);
    }
    CHECK(terminal.stage == TERMINAL_FAILED);
    CHECK(terminal.failed == 1u);

    if (failures == 0) {
        printf("test_tracking_guidance: PASS\n");
        return 0;
    }
    printf("test_tracking_guidance: %d FAILURES\n", failures);
    return 1;
}
