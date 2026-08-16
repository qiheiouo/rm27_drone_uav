#include "trajectory.h"

/* minimum-jerk 五次多项式闭式系数（单轴） */
static Poly5 poly5_min_jerk(float p0, float v0, float a0,
                            float p1, float v1, float a1, float T)
{
    Poly5 p;
    float T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
    float dp = p1 - p0;

    p.c[0] = p0;
    p.c[1] = v0;
    p.c[2] = 0.5f * a0;
    p.c[3] = (20.0f * dp - (8.0f * v1 + 12.0f * v0) * T - (3.0f * a0 - a1) * T2) / (2.0f * T3);
    p.c[4] = (30.0f * -dp + (14.0f * v1 + 16.0f * v0) * T + (3.0f * a0 - 2.0f * a1) * T2) / (2.0f * T4);
    p.c[5] = (12.0f * dp - (6.0f * v1 + 6.0f * v0) * T - (a0 - a1) * T2) / (2.0f * T5);
    return p;
}

static float poly5_eval(const Poly5 *p, float t)
{
    return p->c[0] + t * (p->c[1] + t * (p->c[2] + t * (p->c[3] + t * (p->c[4] + t * p->c[5]))));
}

static float poly5_eval_d1(const Poly5 *p, float t)
{
    return p->c[1] + t * (2.0f * p->c[2] + t * (3.0f * p->c[3] + t * (4.0f * p->c[4] + t * 5.0f * p->c[5])));
}

static float poly5_eval_d2(const Poly5 *p, float t)
{
    return 2.0f * p->c[2] + t * (6.0f * p->c[3] + t * (12.0f * p->c[4] + t * 20.0f * p->c[5]));
}

static void build_segment(PolySegment *seg, Vec3f p0, Vec3f v0,
                          Vec3f p1, float speed, float time_scale)
{
    float dist = vec3_dist(p0, p1);
    float T = time_scale * dist / (speed > 0.1f ? speed : 0.1f);
    if (T < 0.3f) {
        T = 0.3f;   /* 最短段时间，避免数值奇异 */
    }
    seg->px = poly5_min_jerk(p0.x, v0.x, 0.0f, p1.x, 0.0f, 0.0f, T);
    seg->py = poly5_min_jerk(p0.y, v0.y, 0.0f, p1.y, 0.0f, 0.0f, T);
    seg->pz = poly5_min_jerk(p0.z, v0.z, 0.0f, p1.z, 0.0f, 0.0f, T);
    seg->dur = T;
}

void traj_build(Trajectory *traj, Vec3f start_pos, Vec3f start_vel,
                const WaypointQueue *route, float time_scale)
{
    Vec3f pos = start_pos;
    Vec3f vel = start_vel;
    uint8_t i;

    traj->count = 0u;
    traj->duration = 0.0f;

    for (i = route->index; i < route->count && traj->count < TRAJ_MAX_SEGS; i++) {
        PolySegment *seg = &traj->segs[traj->count];
        build_segment(seg, pos, vel, route->items[i].pos, route->items[i].speed, time_scale);
        traj->duration += seg->dur;
        pos = route->items[i].pos;
        vel = vec3_zero();   /* 端点停止 */
        traj->count++;
    }
}

void traj_build_single(Trajectory *traj, Vec3f start_pos, Vec3f start_vel,
                       Vec3f goal_pos, float speed, float time_scale)
{
    traj->count = 1u;
    build_segment(&traj->segs[0], start_pos, start_vel, goal_pos, speed, time_scale);
    traj->duration = traj->segs[0].dur;
}

void traj_evaluate(const Trajectory *traj, float t,
                   Vec3f *pos, Vec3f *vel, Vec3f *accel)
{
    uint8_t i;
    float ts = t;

    if (traj->count == 0u) {
        *pos = vec3_zero();
        *vel = vec3_zero();
        *accel = vec3_zero();
        return;
    }
    if (ts < 0.0f) {
        ts = 0.0f;
    }

    for (i = 0u; i < traj->count; i++) {
        const PolySegment *seg = &traj->segs[i];
        if (ts <= seg->dur || i == traj->count - 1u) {
            float te = ts > seg->dur ? seg->dur : ts;
            pos->x = poly5_eval(&seg->px, te);
            pos->y = poly5_eval(&seg->py, te);
            pos->z = poly5_eval(&seg->pz, te);
            vel->x = poly5_eval_d1(&seg->px, te);
            vel->y = poly5_eval_d1(&seg->py, te);
            vel->z = poly5_eval_d1(&seg->pz, te);
            accel->x = poly5_eval_d2(&seg->px, te);
            accel->y = poly5_eval_d2(&seg->py, te);
            accel->z = poly5_eval_d2(&seg->pz, te);
            return;
        }
        ts -= seg->dur;
    }
}

uint8_t traj_done(const Trajectory *traj, float t)
{
    return (t >= traj->duration) ? 1u : 0u;
}
