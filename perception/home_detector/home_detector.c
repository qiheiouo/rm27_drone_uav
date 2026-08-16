#include "home_detector.h"

void home_detector_init(HomeDetector *det, float filter_cutoff_hz, float dt)
{
    lp_init(&det->fx, filter_cutoff_hz, dt);
    lp_init(&det->fy, filter_cutoff_hz, dt);
    lp_init(&det->fz, filter_cutoff_hz, dt);
    det->lost_time = 1e9f;
    det->out.visible = 0u;
    det->out.rel_pos = vec3_zero();
    det->out.time_since_update = 1e9f;
}

void home_detector_update(HomeDetector *det, const HomeObs *obs, float dt)
{
    if (obs->visible) {
        Vec3f rel;
        rel.x = lp_update(&det->fx, obs->rel_pos.x);
        rel.y = lp_update(&det->fy, obs->rel_pos.y);
        rel.z = lp_update(&det->fz, obs->rel_pos.z);
        det->out.rel_pos = rel;
        det->out.visible = 1u;
        det->lost_time = 0.0f;
    } else {
        det->out.visible = 0u;
        det->lost_time += dt;
    }
    det->out.time_since_update = det->lost_time;
}
