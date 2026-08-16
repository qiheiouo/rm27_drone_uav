/*
 * flight_state.h - 飞控分层接口的状态定义
 *
 * Plan 第 9 节：飞控和导航必须分层。导航层只输出
 * position/velocity/yaw 目标，绝不直接控制电机。
 * 姿态/角速度环与电机混控属于低层飞控（最高优先级），
 * 即使视觉/规划/定位全部失效也必须保持姿态稳定。
 */
#ifndef FLIGHT_STATE_H
#define FLIGHT_STATE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FC_DISARMED = 0,
    FC_ARMED_IDLE,       /* 解锁但未起飞 */
    FC_POSITION_MODE,    /* 接受导航层位置/速度目标 */
    FC_STABILIZE_MODE,   /* 仅姿态稳定（导航失效时的保底） */
    FC_EMERGENCY         /* 紧急（直接降落/断油由低层裁决） */
} FlightControllerMode;

typedef struct {
    FlightControllerMode mode;
    uint8_t  armed;
    float    battery_v;
} FlightState;

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_STATE_H */
