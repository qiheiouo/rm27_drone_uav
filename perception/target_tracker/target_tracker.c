#include "target_tracker.h"

void target_tracker_init(TargetTracker *trk, float filter_cutoff_hz, float dt)
{
    lp_init(&trk->fx, filter_cutoff_hz, dt);
    lp_init(&trk->fy, filter_cutoff_hz, dt);
    lp_init(&trk->fz, filter_cutoff_hz, dt);
    trk->lost_time = 1e9f;
    trk->out.visible = 0u;
    trk->out.rel_pos = vec3_zero();
    trk->out.range = 0.0f;
    trk->out.time_since_update = 1e9f;
}

void target_tracker_reset(TargetTracker *trk)
{
    lp_reset(&trk->fx);
    lp_reset(&trk->fy);
    lp_reset(&trk->fz);
    trk->lost_time = 1e9f;
    trk->out.visible = 0u;
    trk->out.time_since_update = 1e9f;
}

void target_tracker_update(TargetTracker *trk, const TargetObs *obs, float dt)
{
    if (obs->visible) {
        Vec3f rel;
        rel.x = lp_update(&trk->fx, obs->rel_pos.x);
        rel.y = lp_update(&trk->fy, obs->rel_pos.y);
        rel.z = lp_update(&trk->fz, obs->rel_pos.z);
        trk->out.rel_pos = rel;
        trk->out.range = vec3_norm(rel);
        trk->out.visible = 1u;
        trk->lost_time = 0.0f;
    } else {
        trk->out.visible = 0u;
        trk->lost_time += dt;
    }
    trk->out.time_since_update = trk->lost_time;
}
