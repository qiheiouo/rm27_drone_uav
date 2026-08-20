#include "target_tracker.h"

void target_tracker_default_config(TargetTrackerConfig *cfg)
{
    cfg->alpha = 0.55f;
    cfg->beta = 0.12f;
    cfg->max_innovation_m = 2.0f;
    cfg->coast_timeout_s = 0.35f;
    cfg->lost_timeout_s = 1.0f;
    cfg->confidence_decay_per_s = 0.8f;
}

void target_tracker_init_config(TargetTracker *trk, const TargetTrackerConfig *cfg)
{
    trk->cfg = *cfg;
    target_tracker_reset(trk);
}

void target_tracker_init(TargetTracker *trk, float filter_cutoff_hz, float dt)
{
    TargetTrackerConfig cfg;
    float alpha;
    target_tracker_default_config(&cfg);
    alpha = 6.2831853f * filter_cutoff_hz * dt;
    cfg.alpha = clampf(alpha / (1.0f + alpha), 0.15f, 0.85f);
    cfg.beta = clampf(0.25f * cfg.alpha, 0.04f, 0.25f);
    target_tracker_init_config(trk, &cfg);
}

void target_tracker_reset(TargetTracker *trk)
{
    trk->lost_time = 1e9f;
    trk->initialized = 0u;
    trk->out.visible = 0u;
    trk->out.status = TARGET_TRACK_LOST;
    trk->out.rel_pos = vec3_zero();
    trk->out.rel_vel = vec3_zero();
    trk->out.bearing = vec3(1.0f, 0.0f, 0.0f);
    trk->out.range = 0.0f;
    trk->out.confidence = 0.0f;
    trk->out.time_since_update = 1e9f;
    trk->out.age = 0.0f;
    trk->out.timestamp_ms = 0u;
}

void target_tracker_update(TargetTracker *trk, const TargetObs *obs, float dt)
{
    float step = (nav_isfinite(dt) && dt > 1e-4f && dt <= 0.2f) ? dt : 0.01f;
    uint8_t measurement_ok = (obs->visible && vec3_is_finite(obs->rel_pos)) ? 1u : 0u;

    if (trk->initialized) {
        trk->out.rel_pos = vec3_add(trk->out.rel_pos,
                                    vec3_scale(trk->out.rel_vel, step));
    }

    if (measurement_ok) {
        float measurement_confidence = obs->confidence;
        Vec3f innovation;

        if (!nav_isfinite(measurement_confidence) || measurement_confidence <= 0.0f) {
            measurement_confidence = 1.0f;
        }
        measurement_confidence = clampf(measurement_confidence, 0.0f, 1.0f);

        if (!trk->initialized || trk->out.status == TARGET_TRACK_LOST) {
            trk->out.rel_pos = obs->rel_pos;
            trk->out.rel_vel = vec3_zero();
            trk->initialized = 1u;
            innovation = vec3_zero();
        } else {
            innovation = vec3_sub(obs->rel_pos, trk->out.rel_pos);
            if (vec3_norm(innovation) > trk->cfg.max_innovation_m) {
                measurement_ok = 0u;
            }
        }

        if (measurement_ok) {
            trk->out.rel_pos = vec3_add(trk->out.rel_pos,
                                        vec3_scale(innovation, trk->cfg.alpha));
            trk->out.rel_vel = vec3_add(trk->out.rel_vel,
                                        vec3_scale(innovation, trk->cfg.beta / step));
            trk->out.visible = 1u;
            trk->out.status = TARGET_TRACK_TRACKING;
            trk->out.confidence = measurement_confidence;
            trk->out.timestamp_ms = obs->timestamp_ms;
            trk->lost_time = 0.0f;
            trk->out.age += step;
        }
    }

    if (!measurement_ok) {
        trk->out.visible = 0u;
        trk->lost_time += step;
        trk->out.confidence = clampf(trk->out.confidence -
                                     trk->cfg.confidence_decay_per_s * step,
                                     0.0f, 1.0f);
        if (trk->initialized && trk->lost_time < trk->cfg.lost_timeout_s) {
            trk->out.status = TARGET_TRACK_COASTING;
        } else {
            trk->out.status = TARGET_TRACK_LOST;
            trk->out.rel_vel = vec3_zero();
            /* A later observation starts a fresh track instead of being
             * rejected forever by an innovation against stale geometry. */
            trk->initialized = 0u;
        }
    }

    trk->out.range = vec3_norm(trk->out.rel_pos);
    trk->out.bearing = vec3_normalize_or(trk->out.rel_pos,
                                         vec3(1.0f, 0.0f, 0.0f));
    trk->out.time_since_update = trk->lost_time;
}

const char *target_track_status_name(TargetTrackStatus status)
{
    switch (status) {
    case TARGET_TRACK_TRACKING: return "TRACKING";
    case TARGET_TRACK_COASTING: return "COASTING";
    case TARGET_TRACK_LOST:     return "LOST";
    default:                    return "?";
    }
}
