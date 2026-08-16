/*
 * trajectory_message.h - 未来轨迹共享消息（Plan 第 4 节）
 *
 * 参考 EGO-Swarm 的去中心化避碰：广播压缩后的未来轨迹，
 * 而非整张地图（竹林实验：一条轨迹约 170 B，平均约 2 kB/s）。
 * 固定容量、定长编码，适合串口/数传直接发送。
 *
 * 第一版不发送、不接收；仅保留消息格式。
 */
#ifndef TRAJECTORY_MESSAGE_H
#define TRAJECTORY_MESSAGE_H

#include <stdint.h>
#include "nav_math.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SWARM_TRAJ_MAX_POINTS 8u

typedef struct {
    uint8_t  agent_id;
    uint32_t timestamp_ms;
    uint8_t  point_count;                 /* <= SWARM_TRAJ_MAX_POINTS */
    float    dt;                          /* 相邻轨迹点的时间间隔 (s) */
    Vec3f    points[SWARM_TRAJ_MAX_POINTS];
} TrajectoryMessage;

#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_MESSAGE_H */
