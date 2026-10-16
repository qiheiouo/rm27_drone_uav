/* PX4 MAVLink adapter. Protocol transport remains platform-owned. */
#ifndef PX4_MAVLINK_ADAPTER_H
#define PX4_MAVLINK_ADAPTER_H

#include <stdint.h>

#include "fcu_bridge.h"
#include "common/mavlink.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PX4_MAVLINK_IGNORED = 0,
    PX4_MAVLINK_ACCEPTED,
    PX4_MAVLINK_REJECTED
} Px4MavlinkIngestResult;

typedef enum {
    PX4_SETPOINT_ERROR = 0,
    PX4_SETPOINT_DISABLED,
    PX4_SETPOINT_EXPIRED,
    PX4_SETPOINT_READY
} Px4SetpointBuildResult;

typedef struct {
    uint8_t source_system_id;
    uint8_t source_component_id;
    uint8_t target_system_id;
    uint8_t target_component_id;
    uint32_t heartbeat_timeout_ms;
    uint32_t state_timeout_ms;
    uint32_t imu_timeout_ms;
    /* Proper rotation from PX4 local NED vectors into navigation local z-up. */
    Quatf ned_to_nav;
} Px4MavlinkConfig;

typedef struct {
    uint32_t custom_mode;
    uint32_t last_heartbeat_ms;
    uint32_t last_message_ms;
    uint32_t last_command_ack_ms;
    uint16_t last_command_ack;
    uint8_t target_system_id;
    uint8_t target_component_id;
    uint8_t base_mode;
    uint8_t system_status;
    uint8_t landed_state;
    uint8_t last_command_result;
    uint8_t px4_detected;
    uint8_t link_active;
    uint8_t armed;
    uint8_t offboard_active;
} Px4VehicleFeedback;

typedef struct {
    Px4MavlinkConfig cfg;
    FcuStateSnapshot state;
    Px4VehicleFeedback feedback;
    Vec3f imu_accel_flu;
    uint32_t position_received_ms;
    uint32_t attitude_received_ms;
    uint32_t imu_received_ms;
    uint32_t estimator_received_ms;
    uint32_t last_position_boot_ms;
    uint32_t restart_count;
    uint16_t state_sequence;
    uint32_t estimator_flags;
    uint8_t position_seen;
    uint8_t attitude_seen;
    uint8_t imu_seen;
    uint8_t estimator_seen;
} Px4MavlinkAdapter;

void px4_mavlink_config_default(Px4MavlinkConfig *cfg);
uint8_t px4_mavlink_config_valid(const Px4MavlinkConfig *cfg);
void px4_mavlink_init(Px4MavlinkAdapter *adapter,
                      const Px4MavlinkConfig *cfg);

Px4MavlinkIngestResult px4_mavlink_ingest(
    Px4MavlinkAdapter *adapter,
    const mavlink_message_t *message,
    uint32_t received_at_ms);

uint8_t px4_mavlink_get_flight_state(Px4MavlinkAdapter *adapter,
                                     uint32_t now_ms,
                                     FlightState *flight_state,
                                     uint32_t *age_ms);
uint8_t px4_mavlink_get_imu(const Px4MavlinkAdapter *adapter,
                            uint32_t now_ms,
                            ImuSample *imu,
                            uint32_t *age_ms);
void px4_mavlink_get_feedback(const Px4MavlinkAdapter *adapter,
                              uint32_t now_ms,
                              Px4VehicleFeedback *feedback);

Px4SetpointBuildResult px4_mavlink_make_velocity_setpoint(
    const Px4MavlinkAdapter *adapter,
    const FcuSetpoint *setpoint,
    uint32_t now_ms,
    mavlink_message_t *message);

uint8_t px4_mavlink_make_heartbeat(const Px4MavlinkAdapter *adapter,
                                   mavlink_message_t *message);
uint8_t px4_mavlink_make_message_interval_request(
    const Px4MavlinkAdapter *adapter,
    uint32_t message_id,
    int32_t interval_us,
    mavlink_message_t *message);
uint8_t px4_mavlink_make_offboard_request(const Px4MavlinkAdapter *adapter,
                                          mavlink_message_t *message);
uint8_t px4_mavlink_make_position_hold_request(
    const Px4MavlinkAdapter *adapter,
    mavlink_message_t *message);
uint8_t px4_mavlink_make_arm_request(const Px4MavlinkAdapter *adapter,
                                     uint8_t arm,
                                     mavlink_message_t *message);

#ifdef __cplusplus
}
#endif

#endif /* PX4_MAVLINK_ADAPTER_H */
