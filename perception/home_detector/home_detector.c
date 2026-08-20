#include "home_detector.h"

void home_detector_reset(HomeDetector *det)
{
    lp_reset(&det->fx);
    lp_reset(&det->fy);
    lp_reset(&det->fz);
    lp_reset(&det->fyaw);
    det->lost_time = 1e9f;
    det->out.visible = 0u;
    det->out.confidence = 0.0f;
    det->out.rel_pos = vec3_zero();
    det->out.relative_yaw = 0.0f;
    det->out.time_since_update = 1e9f;
    det->out.timestamp_ms = 0u;
}

void home_detector_init(HomeDetector *det, float filter_cutoff_hz, float dt)
{
    lp_init(&det->fx, filter_cutoff_hz, dt);
    lp_init(&det->fy, filter_cutoff_hz, dt);
    lp_init(&det->fz, filter_cutoff_hz, dt);
    lp_init(&det->fyaw, filter_cutoff_hz, dt);
    det->confidence_decay_per_s = 1.0f;
    home_detector_reset(det);
}

void home_detector_update(HomeDetector *det, const HomeObs *obs, float dt)
{
    float step = (nav_isfinite(dt) && dt > 1e-4f && dt <= 0.2f) ? dt : 0.01f;
    if (obs->visible && vec3_is_finite(obs->rel_pos)) {
        float confidence = obs->confidence;
        if (!nav_isfinite(confidence) || confidence <= 0.0f) {
            confidence = 1.0f;
        }
        det->out.rel_pos.x = lp_update(&det->fx, obs->rel_pos.x);
        det->out.rel_pos.y = lp_update(&det->fy, obs->rel_pos.y);
        det->out.rel_pos.z = lp_update(&det->fz, obs->rel_pos.z);
        det->out.relative_yaw = lp_update(&det->fyaw, wrap_pi(obs->relative_yaw));
        det->out.visible = 1u;
        det->out.confidence = clampf(confidence, 0.0f, 1.0f);
        det->out.timestamp_ms = obs->timestamp_ms;
        det->lost_time = 0.0f;
    } else {
        det->out.visible = 0u;
        det->lost_time += step;
        det->out.confidence = clampf(det->out.confidence -
                                     det->confidence_decay_per_s * step,
                                     0.0f, 1.0f);
    }
    det->out.time_since_update = det->lost_time;
}
