/* Fixed-capacity state view for optional decentralized operation. */
#ifndef AGENT_STATE_H
#define AGENT_STATE_H

#include <stdint.h>
#include "nav_math.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SWARM_MAX_OTHER_AGENTS 4u

typedef struct {
    uint8_t  agent_id;
    uint16_t sequence;
    uint32_t timestamp_ms;
    Vec3f    pos;
    Vec3f    vel;
    float    quality;
    float    age_s;
    uint8_t  valid;
} AgentState;

typedef struct {
    AgentState self;
    AgentState others[SWARM_MAX_OTHER_AGENTS];
    uint8_t other_count;
} SwarmView;

static inline void agent_state_clear(AgentState *state)
{
    state->agent_id = 0u;
    state->sequence = 0u;
    state->timestamp_ms = 0u;
    state->pos = vec3_zero();
    state->vel = vec3_zero();
    state->quality = 0.0f;
    state->age_s = 1e9f;
    state->valid = 0u;
}

static inline void swarm_view_init(SwarmView *view, uint8_t self_id)
{
    uint8_t index;
    agent_state_clear(&view->self);
    view->self.agent_id = self_id;
    view->self.valid = 1u;
    view->self.quality = 1.0f;
    for (index = 0u; index < SWARM_MAX_OTHER_AGENTS; index++) {
        agent_state_clear(&view->others[index]);
    }
    view->other_count = 0u;
}

#ifdef __cplusplus
}
#endif

#endif /* AGENT_STATE_H */
