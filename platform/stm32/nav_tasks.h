/* Fixed-memory STM32 adapter for the platform-neutral navigation runtime. */
#ifndef NAV_TASKS_H
#define NAV_TASKS_H

#include <stdint.h>
#include "nav_runtime.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef NavRuntimeConfig NavAppConfig;

typedef struct {
    NavRuntime runtime;
    uint32_t config_errors;
} NavApp;

/* Returns the configuration error mask; zero means initialization succeeded. */
uint32_t nav_app_init(NavApp *app, const NavAppConfig *cfg);

/* Call at 100--200 Hz. All expensive loops have compile-time bounds. */
void nav_app_step(NavApp *app, float dt);

#ifdef __cplusplus
}
#endif

#endif /* NAV_TASKS_H */
