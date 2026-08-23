/* Fixed-memory contract between the navigation MCU and an external FCU. */
#ifndef FCU_BRIDGE_H
#define FCU_BRIDGE_H

#include <stdint.h>

#include "nav_math.h"
#include "state_estimator.h"
#include "pos_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FCU_LINK_SCHEMA_VERSION       1u
#define FCU_LINK_FRAME_TYPE_STATE     1u
#define FCU_LINK_FRAME_TYPE_SETPOINT  2u
#define FCU_LINK_STATE_FRAME_SIZE     100u
#define FCU_LINK_SETPOINT_FRAME_SIZE  72u
#define FCU_LINK_MAX_FRAME_SIZE       FCU_LINK_STATE_FRAME_SIZE

enum {
    FCU_STATE_VALID_ATTITUDE = 1u << 0,
    FCU_STATE_VALID_ANGULAR_VELOCITY = 1u << 1,
    FCU_STATE_VALID_LINEAR_ACCELERATION = 1u << 2,
    FCU_STATE_VALID_LOCAL_POSITION = 1u << 3,
    FCU_STATE_VALID_LOCAL_VELOCITY = 1u << 4,
    FCU_STATE_VALID_HEIGHT = 1u << 5,
    FCU_STATE_VALID_BATTERY = 1u << 6
};

#define FCU_STATE_VALID_KNOWN_MASK ((uint32_t)( \
    FCU_STATE_VALID_ATTITUDE | FCU_STATE_VALID_ANGULAR_VELOCITY | \
    FCU_STATE_VALID_LINEAR_ACCELERATION | FCU_STATE_VALID_LOCAL_POSITION | \
    FCU_STATE_VALID_LOCAL_VELOCITY | FCU_STATE_VALID_HEIGHT | \
    FCU_STATE_VALID_BATTERY))

enum {
    FCU_CAP_POSITION_SETPOINT = 1u << 0,
    FCU_CAP_VELOCITY_SETPOINT = 1u << 1,
    FCU_CAP_ACCELERATION_SETPOINT = 1u << 2,
    FCU_CAP_YAW_SETPOINT = 1u << 3,
    FCU_CAP_YAW_RATE_SETPOINT = 1u << 4,
    FCU_CAP_EXTERNAL_CONTROL = 1u << 5
};

#define FCU_CAP_KNOWN_MASK ((uint32_t)( \
    FCU_CAP_POSITION_SETPOINT | FCU_CAP_VELOCITY_SETPOINT | \
    FCU_CAP_ACCELERATION_SETPOINT | FCU_CAP_YAW_SETPOINT | \
    FCU_CAP_YAW_RATE_SETPOINT | FCU_CAP_EXTERNAL_CONTROL))

enum {
    FCU_STATE_FLAG_ARMED = 1u << 0,
    FCU_STATE_FLAG_EXTERNAL_CONTROL_ACTIVE = 1u << 1,
    FCU_STATE_FLAG_ESTIMATOR_HEALTHY = 1u << 2
};

#define FCU_STATE_FLAG_KNOWN_MASK ((uint16_t)( \
    FCU_STATE_FLAG_ARMED | FCU_STATE_FLAG_EXTERNAL_CONTROL_ACTIVE | \
    FCU_STATE_FLAG_ESTIMATOR_HEALTHY))

enum {
    FCU_SETPOINT_VALID_POSITION = 1u << 0,
    FCU_SETPOINT_VALID_VELOCITY = 1u << 1,
    FCU_SETPOINT_VALID_ACCELERATION = 1u << 2,
    FCU_SETPOINT_VALID_YAW = 1u << 3,
    FCU_SETPOINT_VALID_YAW_RATE = 1u << 4
};

#define FCU_SETPOINT_VALID_KNOWN_MASK ((uint32_t)( \
    FCU_SETPOINT_VALID_POSITION | FCU_SETPOINT_VALID_VELOCITY | \
    FCU_SETPOINT_VALID_ACCELERATION | FCU_SETPOINT_VALID_YAW | \
    FCU_SETPOINT_VALID_YAW_RATE))

typedef enum {
    FCU_CONTROL_AUTO = 0,
    FCU_CONTROL_VELOCITY,
    FCU_CONTROL_ACCELERATION
} FcuControlPreference;

typedef enum {
    FCU_BRIDGE_WAITING = 0,
    FCU_BRIDGE_ACTIVE,
    FCU_BRIDGE_STALE
} FcuBridgeStatus;

typedef enum {
    FCU_BRIDGE_ACCEPTED = 0,
    FCU_BRIDGE_RESYNCHRONIZED,
    FCU_BRIDGE_REJECTED_INVALID,
    FCU_BRIDGE_REJECTED_DUPLICATE,
    FCU_BRIDGE_REJECTED_OUT_OF_ORDER
} FcuBridgeIngestResult;

typedef enum {
    FCU_LINK_OK = 0,
    FCU_LINK_ERROR_ARGUMENT,
    FCU_LINK_ERROR_SIZE,
    FCU_LINK_ERROR_MAGIC,
    FCU_LINK_ERROR_VERSION,
    FCU_LINK_ERROR_TYPE,
    FCU_LINK_ERROR_FLAGS,
    FCU_LINK_ERROR_CRC,
    FCU_LINK_ERROR_VALUE
} FcuLinkResult;

typedef enum {
    FCU_LINK_PARSE_NONE = 0,
    FCU_LINK_PARSE_FRAME,
    FCU_LINK_PARSE_REJECTED
} FcuLinkParseResult;

/* All vectors use the navigation frame: local X/Y and positive Z upward. */
typedef struct {
    uint16_t sequence;
    uint32_t source_timestamp_ms;
    uint32_t valid_mask;
    uint32_t capabilities;
    Quatf attitude;
    Vec3f angular_velocity;
    Vec3f linear_acceleration;
    Vec3f local_position;
    Vec3f local_velocity;
    float height_m;
    float battery_remaining;
    uint8_t armed;
    uint8_t external_control_active;
    uint8_t estimator_healthy;
} FcuStateSnapshot;

typedef struct {
    uint16_t sequence;
    uint32_t source_timestamp_ms;
    uint32_t valid_mask;
    Vec3f position_sp;
    Vec3f velocity_sp;
    Vec3f acceleration_sp;
    float yaw_sp;
    float yaw_rate_sp;
    uint16_t validity_ms;
    FcuControlPreference preferred_control;
    uint8_t motion_enabled;
} FcuSetpoint;

typedef struct {
    uint32_t state_timeout_ms;
    uint16_t command_validity_ms;
    uint32_t required_state_mask;
    FcuControlPreference preferred_control;
} FcuBridgeConfig;

typedef struct {
    uint32_t accepted_states;
    uint32_t resynchronized_states;
    uint32_t invalid_states;
    uint32_t duplicate_states;
    uint32_t out_of_order_states;
    uint32_t stale_transitions;
    uint32_t unsupported_commands;
} FcuBridgeStats;

typedef struct {
    FcuBridgeConfig cfg;
    FcuStateSnapshot latest_state;
    FcuBridgeStats stats;
    uint32_t received_at_ms;
    uint16_t next_command_sequence;
    FcuBridgeStatus status;
    uint8_t state_seen;
    uint8_t stale_latched;
} FcuBridge;

typedef struct {
    uint8_t data[FCU_LINK_MAX_FRAME_SIZE];
    uint16_t size;
} FcuLinkFrame;

typedef struct {
    uint8_t data[FCU_LINK_MAX_FRAME_SIZE];
    uint16_t count;
    uint16_t expected_size;
    uint32_t rejected_headers;
    uint32_t completed_frames;
} FcuLinkParser;

void fcu_bridge_config_default(FcuBridgeConfig *cfg);
uint8_t fcu_bridge_config_valid(const FcuBridgeConfig *cfg);
void fcu_bridge_init(FcuBridge *bridge, const FcuBridgeConfig *cfg);
FcuBridgeIngestResult fcu_bridge_ingest_state(FcuBridge *bridge,
                                               const FcuStateSnapshot *state,
                                               uint32_t received_at_ms);
FcuBridgeStatus fcu_bridge_update(FcuBridge *bridge, uint32_t now_ms);
uint8_t fcu_bridge_get_state(FcuBridge *bridge, uint32_t now_ms,
                             FcuStateSnapshot *state, uint32_t *age_ms);
uint8_t fcu_bridge_make_setpoint(FcuBridge *bridge,
                                 const GuidanceOutput *guidance,
                                 const CtrlOutput *control,
                                 uint8_t motion_enabled,
                                 uint32_t now_ms,
                                 FcuSetpoint *setpoint);

/* Convert a normalized FCU snapshot into inputs consumed by NavRuntime. */
uint8_t fcu_state_to_nav_samples(const FcuStateSnapshot *state,
                                 uint32_t local_timestamp_ms,
                                 ImuSample *imu,
                                 OdomSample *odometry,
                                 float *height_m);

/* Build a protocol-neutral command envelope for a board-specific adapter. */
uint8_t fcu_setpoint_from_nav(const GuidanceOutput *guidance,
                              const CtrlOutput *control,
                              FcuControlPreference preferred_control,
                              uint8_t motion_enabled,
                              uint16_t sequence,
                              uint32_t timestamp_ms,
                              uint16_t validity_ms,
                              FcuSetpoint *setpoint);
void fcu_setpoint_disable(uint16_t sequence, uint32_t timestamp_ms,
                          FcuSetpoint *setpoint);
uint8_t fcu_setpoint_is_fresh(const FcuSetpoint *setpoint,
                              uint32_t received_at_ms, uint32_t now_ms);

FcuLinkResult fcu_link_encode_state(
    const FcuStateSnapshot *state,
    uint8_t frame[FCU_LINK_STATE_FRAME_SIZE]);
FcuLinkResult fcu_link_decode_state(const uint8_t *frame, uint16_t size,
                                    FcuStateSnapshot *state);
FcuLinkResult fcu_link_encode_setpoint(
    const FcuSetpoint *setpoint,
    uint8_t frame[FCU_LINK_SETPOINT_FRAME_SIZE]);
FcuLinkResult fcu_link_decode_setpoint(const uint8_t *frame, uint16_t size,
                                       FcuSetpoint *setpoint);
uint32_t fcu_link_crc32(const uint8_t *data, uint16_t size);

void fcu_link_parser_init(FcuLinkParser *parser);
FcuLinkParseResult fcu_link_parser_push_byte(FcuLinkParser *parser,
                                             uint8_t byte,
                                             FcuLinkFrame *frame);

const char *fcu_bridge_status_name(FcuBridgeStatus status);
const char *fcu_link_result_name(FcuLinkResult result);

#ifdef __cplusplus
}
#endif

#endif /* FCU_BRIDGE_H */
