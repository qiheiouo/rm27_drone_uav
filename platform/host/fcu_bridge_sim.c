/* Deterministic packet-level UART simulation for the protocol-neutral FCU bridge. */
#include <stdio.h>
#include <string.h>

#include "fcu_bridge.h"

#define SIM_DT_MS 10u
#define SIM_DURATION_MS 1600u
#define SIM_UART_DELAY_MS 20u
#define SIM_OUTAGE_START_MS 800u
#define SIM_OUTAGE_END_MS 1100u
#define SIM_QUEUE_CAPACITY 8u
#define SIM_PACKET_CAPACITY (FCU_LINK_MAX_FRAME_SIZE + 2u)

typedef struct {
    uint8_t data[SIM_PACKET_CAPACITY];
    uint16_t size;
    uint32_t deliver_at_ms;
    uint8_t occupied;
} SimPacket;

typedef struct {
    SimPacket packets[SIM_QUEUE_CAPACITY];
    uint32_t enqueued;
    uint32_t dropped;
    uint32_t delivered;
} SimUart;

static uint8_t in_outage(uint32_t now_ms)
{
    return (now_ms >= SIM_OUTAGE_START_MS && now_ms < SIM_OUTAGE_END_MS)
        ? 1u : 0u;
}

static uint8_t uart_enqueue(SimUart *uart, const uint8_t *data, uint16_t size,
                            uint32_t deliver_at_ms)
{
    uint8_t index;
    if (size > SIM_PACKET_CAPACITY) return 0u;
    for (index = 0u; index < SIM_QUEUE_CAPACITY; index++) {
        if (!uart->packets[index].occupied) {
            memcpy(uart->packets[index].data, data, size);
            uart->packets[index].size = size;
            uart->packets[index].deliver_at_ms = deliver_at_ms;
            uart->packets[index].occupied = 1u;
            uart->enqueued++;
            return 1u;
        }
    }
    uart->dropped++;
    return 0u;
}

static uint8_t uart_pop_due(SimUart *uart, uint32_t now_ms,
                            uint8_t *data, uint16_t *size)
{
    uint8_t index;
    uint8_t selected = SIM_QUEUE_CAPACITY;
    uint32_t earliest = 0u;
    for (index = 0u; index < SIM_QUEUE_CAPACITY; index++) {
        if (uart->packets[index].occupied &&
            (int32_t)(now_ms - uart->packets[index].deliver_at_ms) >= 0 &&
            (selected == SIM_QUEUE_CAPACITY ||
             (int32_t)(uart->packets[index].deliver_at_ms - earliest) < 0)) {
            selected = index;
            earliest = uart->packets[index].deliver_at_ms;
        }
    }
    if (selected == SIM_QUEUE_CAPACITY) return 0u;
    memcpy(data, uart->packets[selected].data, uart->packets[selected].size);
    *size = uart->packets[selected].size;
    uart->packets[selected].occupied = 0u;
    uart->delivered++;
    return 1u;
}

static FcuStateSnapshot make_state(uint16_t sequence, uint32_t now_ms)
{
    FcuStateSnapshot state;
    memset(&state, 0, sizeof(state));
    state.sequence = sequence;
    state.source_timestamp_ms = now_ms;
    state.valid_mask = FCU_STATE_VALID_ATTITUDE |
        FCU_STATE_VALID_ANGULAR_VELOCITY |
        FCU_STATE_VALID_LINEAR_ACCELERATION |
        FCU_STATE_VALID_LOCAL_POSITION |
        FCU_STATE_VALID_LOCAL_VELOCITY |
        FCU_STATE_VALID_HEIGHT | FCU_STATE_VALID_BATTERY;
    state.capabilities = FCU_CAP_VELOCITY_SETPOINT |
        FCU_CAP_YAW_SETPOINT | FCU_CAP_EXTERNAL_CONTROL;
    state.attitude = quat_identity();
    state.linear_acceleration = vec3(0.0f, 0.0f, NAV_GRAVITY);
    state.height_m = 1.0f;
    state.battery_remaining = 0.8f;
    state.armed = 1u;
    state.external_control_active = 1u;
    state.estimator_healthy = 1u;
    return state;
}

static uint8_t parser_feed(FcuLinkParser *parser, const uint8_t *data,
                           uint16_t size, FcuLinkFrame *frame)
{
    uint16_t index;
    uint8_t complete = 0u;
    for (index = 0u; index < size; index++) {
        if (fcu_link_parser_push_byte(parser, data[index], frame) ==
            FCU_LINK_PARSE_FRAME) {
            complete = 1u;
        }
    }
    return complete;
}

int main(void)
{
    FcuBridgeConfig cfg;
    FcuBridge bridge;
    FcuLinkParser state_parser;
    FcuLinkParser command_parser;
    SimUart state_uart;
    SimUart command_uart;
    FcuSetpoint outgoing_command;
    FcuSetpoint received_command;
    GuidanceOutput guidance;
    CtrlOutput control;
    uint8_t state_frame[FCU_LINK_STATE_FRAME_SIZE];
    uint8_t command_frame[FCU_LINK_SETPOINT_FRAME_SIZE];
    uint8_t packet[SIM_PACKET_CAPACITY];
    uint16_t packet_size;
    uint16_t state_sequence = 0u;
    uint32_t now_ms;
    uint32_t command_received_at_ms = 0u;
    uint32_t state_decode_errors = 0u;
    uint32_t command_decode_errors = 0u;
    uint32_t state_outage_drops = 0u;
    uint32_t command_outage_drops = 0u;
    uint32_t stale_steps = 0u;
    uint32_t failsafe_steps = 0u;
    uint32_t fresh_command_steps = 0u;
    uint32_t max_state_age_ms = 0u;
    uint8_t command_seen = 0u;
    uint8_t stale_seen = 0u;
    uint8_t failsafe_seen = 0u;
    uint8_t recovered_seen = 0u;
    FcuBridgeStatus previous_status = FCU_BRIDGE_WAITING;

    memset(&state_uart, 0, sizeof(state_uart));
    memset(&command_uart, 0, sizeof(command_uart));
    memset(&received_command, 0, sizeof(received_command));
    fcu_link_parser_init(&state_parser);
    fcu_link_parser_init(&command_parser);
    fcu_bridge_config_default(&cfg);
    cfg.state_timeout_ms = 50u;
    cfg.command_validity_ms = 100u;
    fcu_bridge_init(&bridge, &cfg);

    guidance.pos_sp = vec3_zero();
    guidance.vel_sp = vec3(0.6f, 0.0f, 0.0f);
    guidance.accel_sp = vec3_zero();
    guidance.yaw_sp = 0.0f;
    guidance.use_pos_sp = 0u;
    guidance.use_accel_sp = 0u;
    control.accel_cmd = vec3(0.2f, 0.0f, 0.0f);
    control.yaw_rate_cmd = 0.0f;

    printf("# deterministic dual-MCU FCU UART bridge simulation\n");
    for (now_ms = 0u; now_ms <= SIM_DURATION_MS; now_ms += SIM_DT_MS) {
        FcuStateSnapshot outgoing_state = make_state(state_sequence++, now_ms);
        FcuLinkFrame parsed_frame;
        FcuBridgeStatus status;
        uint32_t state_age_ms = 0u;

        if (fcu_link_encode_state(&outgoing_state, state_frame) != FCU_LINK_OK)
            return 2;
        if (now_ms == 500u) state_frame[70] ^= 0x40u;
        if (in_outage(now_ms)) {
            state_outage_drops++;
        } else if (now_ms == 400u) {
            packet[0] = 0x55u;
            memcpy(&packet[1], state_frame, sizeof(state_frame));
            if (!uart_enqueue(&state_uart, packet,
                              (uint16_t)(sizeof(state_frame) + 1u),
                              now_ms + SIM_UART_DELAY_MS)) return 3;
        } else if (!uart_enqueue(&state_uart, state_frame, sizeof(state_frame),
                                 now_ms + SIM_UART_DELAY_MS)) {
            return 3;
        }

        while (uart_pop_due(&state_uart, now_ms, packet, &packet_size)) {
            if (parser_feed(&state_parser, packet, packet_size, &parsed_frame)) {
                FcuStateSnapshot decoded;
                FcuLinkResult result = fcu_link_decode_state(
                    parsed_frame.data, parsed_frame.size, &decoded);
                if (result == FCU_LINK_OK) {
                    (void)fcu_bridge_ingest_state(&bridge, &decoded, now_ms);
                } else {
                    state_decode_errors++;
                }
            }
        }

        status = fcu_bridge_update(&bridge, now_ms);
        if (previous_status == FCU_BRIDGE_STALE &&
            status == FCU_BRIDGE_ACTIVE) {
            printf("[t=%.2f] EVENT FCU state link recovered\n",
                   (double)now_ms * 0.001);
        }
        if (status == FCU_BRIDGE_ACTIVE) {
            FcuStateSnapshot current;
            if (fcu_bridge_get_state(&bridge, now_ms, &current,
                                     &state_age_ms)) {
                if (state_age_ms > max_state_age_ms) max_state_age_ms = state_age_ms;
            }
            if (fcu_bridge_make_setpoint(&bridge, &guidance, &control, 1u,
                                         now_ms, &outgoing_command) &&
                fcu_link_encode_setpoint(&outgoing_command, command_frame) ==
                    FCU_LINK_OK) {
                if (in_outage(now_ms)) {
                    command_outage_drops++;
                } else if (!uart_enqueue(&command_uart, command_frame,
                                         sizeof(command_frame),
                                         now_ms + SIM_UART_DELAY_MS)) {
                    return 4;
                }
            }
        } else if (status == FCU_BRIDGE_STALE) {
            stale_steps++;
            stale_seen = 1u;
            if (previous_status != FCU_BRIDGE_STALE)
                printf("[t=%.2f] EVENT FCU state stale; navigation output inhibited\n",
                       (double)now_ms * 0.001);
        }

        while (uart_pop_due(&command_uart, now_ms, packet, &packet_size)) {
            if (parser_feed(&command_parser, packet, packet_size,
                            &parsed_frame)) {
                FcuSetpoint decoded;
                FcuLinkResult result = fcu_link_decode_setpoint(
                    parsed_frame.data, parsed_frame.size, &decoded);
                if (result == FCU_LINK_OK) {
                    received_command = decoded;
                    command_received_at_ms = now_ms;
                    command_seen = 1u;
                } else {
                    command_decode_errors++;
                }
            }
        }

        if (command_seen && fcu_setpoint_is_fresh(
                &received_command, command_received_at_ms, now_ms)) {
            fresh_command_steps++;
            if (failsafe_seen && now_ms >= SIM_OUTAGE_END_MS) {
                recovered_seen = 1u;
            }
        } else if (command_seen) {
            failsafe_steps++;
            if (!failsafe_seen) {
                failsafe_seen = 1u;
                printf("[t=%.2f] EVENT FCU command expired; receiver failsafe active\n",
                       (double)now_ms * 0.001);
            }
        }
        previous_status = status;
    }

    printf("state accepted=%lu resync=%lu crc_errors=%lu outage_drops=%lu "
           "stale_steps=%lu max_age_ms=%lu\n",
           (unsigned long)bridge.stats.accepted_states,
           (unsigned long)bridge.stats.resynchronized_states,
           (unsigned long)state_decode_errors,
           (unsigned long)state_outage_drops,
           (unsigned long)stale_steps,
           (unsigned long)max_state_age_ms);
    printf("command fresh_steps=%lu failsafe_steps=%lu decode_errors=%lu "
           "outage_drops=%lu\n",
           (unsigned long)fresh_command_steps,
           (unsigned long)failsafe_steps,
           (unsigned long)command_decode_errors,
           (unsigned long)command_outage_drops);

    if (!stale_seen || !failsafe_seen || !recovered_seen ||
        state_decode_errors != 1u || bridge.stats.stale_transitions != 1u ||
        bridge.stats.resynchronized_states != 1u ||
        fresh_command_steps == 0u || state_uart.dropped != 0u ||
        command_uart.dropped != 0u) {
        printf("FCU_BRIDGE_SIM_FAILED expectations not met\n");
        return 1;
    }
    printf("FCU_BRIDGE_SIM_SUCCESS\n");
    return 0;
}
