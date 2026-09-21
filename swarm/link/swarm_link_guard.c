#include "swarm_link_guard.h"

static float valid_step(float dt)
{
    return (nav_isfinite(dt) && dt > 0.0f && dt <= 0.2f) ? dt : 0.01f;
}

static uint8_t peer_present(const SwarmView *view, uint8_t agent_id)
{
    uint8_t index;
    if (view == 0 || agent_id == 0u) return 0u;
    for (index = 0u; index < view->other_count &&
         index < SWARM_MAX_OTHER_AGENTS; index++) {
        if (view->others[index].valid &&
            view->others[index].agent_id == agent_id) {
            return 1u;
        }
    }
    return 0u;
}

static void enter_state(SwarmLinkGuard *guard, SwarmLinkGuardState state)
{
    if (guard->state != state) {
        guard->state = state;
        guard->state_time_s = 0.0f;
    }
}

static void clear_guard(SwarmLinkGuard *guard)
{
    enter_state(guard, SWARM_LINK_GUARD_CLEAR);
    guard->hazard_agent_id = 0u;
    guard->clear_time_s = 0.0f;
    guard->recovery_time_s = 0.0f;
}

static void begin_monitoring(SwarmLinkGuard *guard, uint8_t agent_id)
{
    uint8_t hazard_changed =
        guard->hazard_agent_id != agent_id ? 1u : 0u;
    guard->hazard_agent_id = agent_id;
    guard->clear_time_s = 0.0f;
    guard->recovery_time_s = 0.0f;
    enter_state(guard, SWARM_LINK_GUARD_MONITORING);
    if (hazard_changed) guard->state_time_s = 0.0f;
}

static void begin_holding(SwarmLinkGuard *guard, const NavState *nav,
                          uint8_t capture_position)
{
    if (capture_position) guard->hold_position = nav->pos;
    guard->clear_time_s = 0.0f;
    guard->recovery_time_s = 0.0f;
    enter_state(guard, SWARM_LINK_GUARD_HOLDING);
}

static void publish_output(const SwarmLinkGuard *guard,
                           SwarmLinkGuardState previous_state,
                           uint8_t present,
                           SwarmLinkGuardOutput *output)
{
    output->state = guard->state;
    output->hold_position = guard->hold_position;
    output->state_time_s = guard->state_time_s;
    output->hazard_agent_id = guard->hazard_agent_id;
    output->active = (guard->state == SWARM_LINK_GUARD_HOLDING ||
                      guard->state == SWARM_LINK_GUARD_RECOVERING) ? 1u : 0u;
    output->peer_present = present;
    output->state_changed = guard->state != previous_state ? 1u : 0u;
}

void swarm_link_guard_default_config(SwarmLinkGuardConfig *cfg)
{
    cfg->conflict_clear_confirm_s = 0.30f;
    cfg->recovery_confirm_s = 0.30f;
    cfg->enabled = 0u;
}

void swarm_link_guard_init(SwarmLinkGuard *guard,
                           const SwarmLinkGuardConfig *cfg)
{
    guard->cfg = *cfg;
    guard->state = SWARM_LINK_GUARD_CLEAR;
    guard->hold_position = vec3_zero();
    guard->state_time_s = 0.0f;
    guard->clear_time_s = 0.0f;
    guard->recovery_time_s = 0.0f;
    guard->hazard_agent_id = 0u;
}

void swarm_link_guard_update(SwarmLinkGuard *guard,
                             const SwarmView *view,
                             const CollisionReport *collision,
                             const NavState *nav,
                             float dt,
                             SwarmLinkGuardOutput *output)
{
    SwarmLinkGuardState previous_state = guard->state;
    float step = valid_step(dt);
    uint8_t present;

    if (!guard->cfg.enabled || view == 0 || collision == 0 || nav == 0) {
        clear_guard(guard);
        publish_output(guard, previous_state, 0u, output);
        return;
    }

    guard->state_time_s += step;
    present = peer_present(view, guard->hazard_agent_id);

    if (collision->conflict && collision->other_agent_id != 0u &&
        !((guard->state == SWARM_LINK_GUARD_HOLDING ||
           guard->state == SWARM_LINK_GUARD_RECOVERING) &&
          guard->hazard_agent_id != collision->other_agent_id)) {
        if (guard->hazard_agent_id != collision->other_agent_id) {
            begin_monitoring(guard, collision->other_agent_id);
            present = 1u;
        } else if (guard->state == SWARM_LINK_GUARD_HOLDING) {
            guard->recovery_time_s = 0.0f;
            enter_state(guard, SWARM_LINK_GUARD_RECOVERING);
            present = 1u;
        } else if (guard->state == SWARM_LINK_GUARD_RECOVERING) {
            guard->recovery_time_s += step;
            present = 1u;
            if (guard->recovery_time_s >= guard->cfg.recovery_confirm_s) {
                begin_monitoring(guard, collision->other_agent_id);
            }
        } else {
            begin_monitoring(guard, collision->other_agent_id);
            present = 1u;
        }
    } else {
        switch (guard->state) {
        case SWARM_LINK_GUARD_MONITORING:
            if (!present) {
                begin_holding(guard, nav, 1u);
            } else {
                guard->clear_time_s += step;
                if (guard->clear_time_s >=
                    guard->cfg.conflict_clear_confirm_s) {
                    clear_guard(guard);
                }
            }
            break;
        case SWARM_LINK_GUARD_HOLDING:
            if (present) {
                guard->recovery_time_s = 0.0f;
                enter_state(guard, SWARM_LINK_GUARD_RECOVERING);
            }
            break;
        case SWARM_LINK_GUARD_RECOVERING:
            if (!present) {
                begin_holding(guard, nav, 0u);
            } else {
                guard->recovery_time_s += step;
                if (guard->recovery_time_s >= guard->cfg.recovery_confirm_s) {
                    clear_guard(guard);
                }
            }
            break;
        case SWARM_LINK_GUARD_CLEAR:
        default:
            break;
        }
    }

    present = peer_present(view, guard->hazard_agent_id);
    publish_output(guard, previous_state, present, output);
}

const char *swarm_link_guard_state_name(SwarmLinkGuardState state)
{
    switch (state) {
    case SWARM_LINK_GUARD_CLEAR: return "CLEAR";
    case SWARM_LINK_GUARD_MONITORING: return "MONITORING";
    case SWARM_LINK_GUARD_HOLDING: return "HOLDING";
    case SWARM_LINK_GUARD_RECOVERING: return "RECOVERING";
    default: return "?";
    }
}
