/*
 * waypoint.h - 固定容量航点队列（Plan 第一版"规划"）
 *
 * 第一版不做 MINCO / 时空联合优化，只做：
 *   waypoint + 速度限制的位置/速度目标生成。
 * 已知场地假设下，航线由预定义 corridor/waypoint graph 给出。
 */
#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <stdint.h>
#include "nav_math.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WP_QUEUE_MAX 8u

typedef struct {
    Vec3f pos;      /* 航点位置 (m) */
    float speed;    /* 飞向该航点的期望速度 (m/s) */
} Waypoint;

typedef struct {
    Waypoint items[WP_QUEUE_MAX];
    uint8_t count;
    uint8_t index;
} WaypointQueue;

void  wq_init(WaypointQueue *q);
/* 返回 0 成功，-1 队列已满 */
int   wq_push(WaypointQueue *q, Vec3f pos, float speed);
/* 队列耗尽时返回最后一个航点（保持悬停目标），空队列返回 0 */
const Waypoint *wq_current(const WaypointQueue *q);
uint8_t wq_done(const WaypointQueue *q);
void  wq_advance(WaypointQueue *q);
void  wq_reset(WaypointQueue *q);
/* 到达当前航点（容差 tol 内）则推进；返回是否因此耗尽队列 */
uint8_t wq_advance_if_reached(WaypointQueue *q, Vec3f pos, float tol);

#ifdef __cplusplus
}
#endif

#endif /* WAYPOINT_H */
