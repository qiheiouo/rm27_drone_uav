/* Platform-neutral fused flight-state input contract. */
#ifndef FLIGHT_STATE_H
#define FLIGHT_STATE_H

#include <stdint.h>

#include "nav_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * All producers must convert into this frame before publishing a sample.
 * It is right-handed, uses metres/seconds, has positive Z upward, and keeps
 * one stable local X/Y origin and heading for the complete navigation run.
 */
typedef enum {
    FLIGHT_STATE_FRAME_LOCAL_XY_Z_UP = 0
} FlightStateFrame;

enum {
    FLIGHT_STATE_VALID_POSITION = 1u << 0,
    FLIGHT_STATE_VALID_VELOCITY = 1u << 1,
    FLIGHT_STATE_VALID_ATTITUDE = 1u << 2
};

#define FLIGHT_STATE_REQUIRED_MASK ((uint32_t)( \
    FLIGHT_STATE_VALID_POSITION | FLIGHT_STATE_VALID_VELOCITY | \
    FLIGHT_STATE_VALID_ATTITUDE))

/*
 * Fused state, not a raw sensor measurement. attitude_body_to_nav is a WXYZ
 * quaternion rotating body-frame vectors into the navigation frame.
 * timestamp_ms is the sample time on NavRuntime's monotonic millisecond clock.
 * valid == 0 reports localization loss even if the last fields are present.
 * armed and external_control_active are feedback from the flight controller,
 * not host command or authorization state. They are consumed only when
 * control_feedback_valid is set.
 */
typedef struct {
    Vec3f position_m;
    Vec3f velocity_mps;
    Quatf attitude_body_to_nav;
    uint32_t timestamp_ms;
    uint32_t valid_mask;
    FlightStateFrame frame;
    uint8_t armed;
    uint8_t external_control_active;
    uint8_t control_feedback_valid;
    uint8_t valid;
} FlightState;

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_STATE_H */
