#include "collision_interface.h"

void collision_init(CollisionConfig *cfg, float safe_separation_m)
{
    cfg->safe_separation_m = safe_separation_m;
}

CollisionReport collision_check(const CollisionConfig *cfg,
                                const Vec3f *self_points, uint8_t self_point_count,
                                const SwarmView *swarm)
{
    CollisionReport rep;
    uint8_t i, j;

    rep.conflict = 0u;
    rep.other_agent_id = 0u;
    rep.min_separation = 1e9f;

    /* 第一版 other_count == 0，直接返回无冲突 */
    for (i = 0u; i < swarm->other_count && i < SWARM_MAX_OTHER_AGENTS; i++) {
        for (j = 0u; j < self_point_count; j++) {
            float d = vec3_dist(self_points[j], swarm->others[i].pos);
            if (d < rep.min_separation) {
                rep.min_separation = d;
                if (d < cfg->safe_separation_m) {
                    rep.conflict = 1u;
                    rep.other_agent_id = swarm->others[i].agent_id;
                }
            }
        }
    }
    return rep;
}
