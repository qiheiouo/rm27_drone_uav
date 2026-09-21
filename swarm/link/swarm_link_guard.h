/* Bounded hold policy when a recently conflicting peer disappears. */
#ifndef SWARM_LINK_GUARD_H
#define SWARM_LINK_GUARD_H

#include <stdint.h>

#include "collision_interface.h"
#include "state_estimator.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SWARM_LINK_GUARD_CLEAR = 0,
    SWARM_LINK_GUARD_MONITORING,
    SWARM_LINK_GUARD_HOLDING,
    SWARM_LINK_GUARD_RECOVERING
} SwarmLinkGuardState;

typedef struct {
    float conflict_clear_confirm_s;
    float recovery_confirm_s;
    uint8_t enabled;
} SwarmLinkGuardConfig;

typedef struct {
    SwarmLinkGuardState state;
    Vec3f hold_position;
    float state_time_s;
    uint8_t hazard_agent_id;
    uint8_t active;
    uint8_t peer_present;
    uint8_t state_changed;
} SwarmLinkGuardOutput;

typedef struct {
    SwarmLinkGuardConfig cfg;
    SwarmLinkGuardState state;
    Vec3f hold_position;
    float state_time_s;
    float clear_time_s;
    float recovery_time_s;
    uint8_t hazard_agent_id;
} SwarmLinkGuard;

void swarm_link_guard_default_config(SwarmLinkGuardConfig *cfg);
void swarm_link_guard_init(SwarmLinkGuard *guard,
                           const SwarmLinkGuardConfig *cfg);
void swarm_link_guard_update(SwarmLinkGuard *guard,
                             const SwarmView *view,
                             const CollisionReport *collision,
                             const NavState *nav,
                             float dt,
                             SwarmLinkGuardOutput *output);
const char *swarm_link_guard_state_name(SwarmLinkGuardState state);

#ifdef __cplusplus
}
#endif

#endif /* SWARM_LINK_GUARD_H */
