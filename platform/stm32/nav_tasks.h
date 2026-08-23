/* Fixed-memory STM32 adapter for the platform-neutral navigation runtime. */
#ifndef NAV_TASKS_H
#define NAV_TASKS_H

#include <stdint.h>
#include "nav_runtime.h"
#include "swarm_link.h"

#ifndef NAV_APP_TELEMETRY_PERIOD_STEPS
#define NAV_APP_TELEMETRY_PERIOD_STEPS 10u
#endif

#ifndef NAV_APP_SWARM_RX_BUDGET
#define NAV_APP_SWARM_RX_BUDGET 4u
#endif

#ifndef NAV_APP_SWARM_TX_PERIOD_STEPS
#define NAV_APP_SWARM_TX_PERIOD_STEPS 5u
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef NavRuntimeConfig NavAppConfig;

typedef struct {
    NavRuntime runtime;
    uint32_t config_errors;
    uint32_t next_log_sequence;
    uint32_t telemetry_sequence;
    uint32_t telemetry_step_count;
    SwarmPeerRegistry swarm_peers;
    uint32_t swarm_decode_errors;
    uint32_t swarm_tx_drops;
    uint32_t swarm_tx_step_count;
    uint16_t swarm_tx_sequence;
} NavApp;

/* Returns the configuration error mask; zero means initialization succeeded. */
uint32_t nav_app_init(NavApp *app, const NavAppConfig *cfg);

/* Call at 100--200 Hz. All expensive loops have compile-time bounds. */
void nav_app_step(NavApp *app, float dt);

#ifdef __cplusplus
}
#endif

#endif /* NAV_TASKS_H */
