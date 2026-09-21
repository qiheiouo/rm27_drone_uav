/* Fixed-memory STM32 adapter for the platform-neutral navigation runtime. */
#ifndef NAV_TASKS_H
#define NAV_TASKS_H

#include <stdint.h>
#include "nav_runtime.h"

#ifndef NAV_APP_TELEMETRY_PERIOD_STEPS
#define NAV_APP_TELEMETRY_PERIOD_STEPS 10u
#endif

#ifndef NAV_APP_FCU_COMMAND_VALIDITY_MS
#define NAV_APP_FCU_COMMAND_VALIDITY_MS 100u
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
    uint32_t fcu_tx_drops;
    uint16_t fcu_command_sequence;
} NavApp;

/* Returns the configuration error mask; zero means initialization succeeded. */
uint32_t nav_app_init(NavApp *app, const NavAppConfig *cfg);

/* Call at 100--200 Hz. All expensive loops have compile-time bounds. */
void nav_app_step(NavApp *app, float dt);

#ifdef __cplusplus
}
#endif

#endif /* NAV_TASKS_H */
