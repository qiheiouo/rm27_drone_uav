#include "waypoint.h"

void wq_init(WaypointQueue *q)
{
    q->count = 0u;
    q->index = 0u;
}

int wq_push(WaypointQueue *q, Vec3f pos, float speed)
{
    if (q->count >= WP_QUEUE_MAX) {
        return -1;
    }
    q->items[q->count].pos = pos;
    q->items[q->count].speed = speed;
    q->count++;
    return 0;
}

const Waypoint *wq_current(const WaypointQueue *q)
{
    if (q->count == 0u) {
        return 0;
    }
    if (q->index >= q->count) {
        return &q->items[q->count - 1u];
    }
    return &q->items[q->index];
}

uint8_t wq_done(const WaypointQueue *q)
{
    return (q->count > 0u && q->index >= q->count) ? 1u : 0u;
}

void wq_advance(WaypointQueue *q)
{
    if (q->index < q->count) {
        q->index++;
    }
}

void wq_reset(WaypointQueue *q)
{
    q->index = 0u;
}

uint8_t wq_advance_if_reached(WaypointQueue *q, Vec3f pos, float tol)
{
    const Waypoint *cur = wq_current(q);
    if (cur == 0) {
        return 1u;
    }
    if (vec3_dist(pos, cur->pos) <= tol) {
        wq_advance(q);
    }
    return wq_done(q);
}
