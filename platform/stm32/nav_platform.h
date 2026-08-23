/* Board-support contract for the STM32 integration.
 *
 * The algorithm library owns no peripherals and performs no allocation. The
 * board project implements these functions and keeps the low-level attitude,
 * rate and motor loops at a higher priority than nav_app_step().
 */
#ifndef NAV_PLATFORM_H
#define NAV_PLATFORM_H

#include <stdint.h>
#include "state_estimator.h"
#include "camera.h"
#include "vision_frontend.h"
#include "pos_controller.h"
#include "obstacle_avoidance.h"
#include "collision_interface.h"
#include "nav_telemetry.h"
#include "swarm_link.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t start;
    uint8_t request_return;
    uint8_t request_emergency;
} NavMissionCommand;

typedef struct {
    uint8_t contact;
    uint8_t charging;
    uint8_t wireless_charge_ready;
} NavDockStatus;

/* Monotonic system time in milliseconds. */
uint32_t nav_time_ms(void);

/* Return 0 for a fresh valid sample, negative when unavailable. */
int nav_imu_read(ImuSample *out);
int nav_tof_read(float *height_m);
int nav_flow_read(FlowFrame *out);
int nav_camera_target_read(PixelObs *out);
int nav_camera_home_read(PixelObs *out);
int nav_local_obstacles_read(DynamicObstacleSet *out);
int nav_mission_command_read(NavMissionCommand *out);
int nav_dock_status_read(NavDockStatus *out);

/*
 * Optional non-blocking swarm transport. Read returns 0 after copying one
 * complete frame, or negative when no frame is available. Write returns 0
 * when the frame was copied/enqueued, or negative when unavailable/full.
 */
int nav_swarm_frame_read(uint8_t *frame, uint16_t capacity, uint16_t *size);
int nav_swarm_frame_write(const uint8_t *frame, uint16_t size);

/* Desired acceleration and yaw rate passed to the existing flight controller. */
void nav_fcu_send(const CtrlOutput *cmd);
void nav_fcu_set_armed(uint8_t armed);
void nav_log(const char *msg);

/*
 * Optional low-priority telemetry sink. It must copy/enqueue the complete
 * frame before returning and must never block the navigation task. Return 0
 * when accepted or negative when the queue is full/unavailable.
 */
int nav_telemetry_write(const uint8_t *frame, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* NAV_PLATFORM_H */
