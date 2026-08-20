#include "impact_detector.h"

void impact_detector_default_fusion_config(ImpactFusionConfig *cfg)
{
    cfg->velocity_jump_threshold_mps = 0.8f;
    cfg->attitude_jump_threshold_rad = 0.35f;
    cfg->debounce_s = 0.01f;
    cfg->refractory_s = 0.5f;
    cfg->minimum_confidence = 0.45f;
}

void impact_detector_init(ImpactDetector *det, const ImpactDetectorConfig *cfg)
{
    det->cfg = *cfg;
    impact_detector_default_fusion_config(&det->fusion);
    det->armed = 1u;
    det->have_previous_motion = 0u;
    det->previous_velocity = vec3_zero();
    det->previous_attitude = quat_identity();
    det->previous_timestamp_ms = 0u;
    impact_detector_reset(det);
}

void impact_detector_configure_fusion(ImpactDetector *det,
                                      const ImpactFusionConfig *cfg)
{
    det->fusion = *cfg;
}

void impact_detector_arm(ImpactDetector *det, uint8_t armed)
{
    det->armed = armed ? 1u : 0u;
    if (!det->armed) {
        det->confirm_count = 0u;
        det->candidate_time = 0.0f;
        det->report.state = NO_IMPACT;
        det->report.confidence = 0.0f;
    }
}

void impact_detector_reset(ImpactDetector *det)
{
    det->triggered = 0u;
    det->confirm_count = 0u;
    det->candidate_time = 0.0f;
    det->refractory_remaining = 0.0f;
    det->report.state = NO_IMPACT;
    det->report.confidence = 0.0f;
    det->report.accel_evidence = 0u;
    det->report.gyro_evidence = 0u;
    det->report.velocity_evidence = 0u;
    det->report.attitude_evidence = 0u;
    det->report.rising_edge = 0u;
}

static float attitude_delta(Quatf previous, Quatf current)
{
    Quatf delta = quat_mul(quat_conj(previous), current);
    float w = fabsf(delta.w);
    return 2.0f * acosf(clampf(w, -1.0f, 1.0f));
}

ImpactReport impact_detector_update_fused(ImpactDetector *det,
                                          const ImuSample *sample,
                                          Vec3f velocity,
                                          Quatf attitude,
                                          float dt)
{
    float step = (nav_isfinite(dt) && dt > 1e-5f && dt <= 0.2f) ? dt : 0.01f;
    float accel_deviation = fabsf(vec3_norm(sample->accel) - NAV_GRAVITY);
    float gyro_magnitude = vec3_norm(sample->gyro);
    float velocity_jump = 0.0f;
    float attitude_jump = 0.0f;
    float score = 0.0f;
    uint8_t evidence_count = 0u;

    det->report.rising_edge = 0u;
    if (det->refractory_remaining > 0.0f) {
        det->refractory_remaining -= step;
        if (det->refractory_remaining < 0.0f) {
            det->refractory_remaining = 0.0f;
        }
    }

    det->report.accel_evidence =
        (accel_deviation >= det->cfg.accel_spike_threshold) ? 1u : 0u;
    det->report.gyro_evidence =
        (gyro_magnitude >= det->cfg.gyro_spike_threshold) ? 1u : 0u;

    if (det->have_previous_motion && vec3_is_finite(velocity)) {
        velocity_jump = vec3_dist(velocity, det->previous_velocity);
        attitude_jump = attitude_delta(det->previous_attitude, attitude);
    }
    det->report.velocity_evidence =
        (velocity_jump >= det->fusion.velocity_jump_threshold_mps) ? 1u : 0u;
    det->report.attitude_evidence =
        (attitude_jump >= det->fusion.attitude_jump_threshold_rad) ? 1u : 0u;

    if (det->report.accel_evidence) {
        score += clampf(accel_deviation / (2.0f * det->cfg.accel_spike_threshold),
                        0.25f, 1.0f);
        evidence_count++;
    }
    if (det->report.gyro_evidence) {
        score += clampf(gyro_magnitude / (2.0f * det->cfg.gyro_spike_threshold),
                        0.25f, 1.0f);
        evidence_count++;
    }
    if (det->report.velocity_evidence) {
        score += clampf(velocity_jump /
                        (2.0f * det->fusion.velocity_jump_threshold_mps), 0.25f, 1.0f);
        evidence_count++;
    }
    if (det->report.attitude_evidence) {
        score += clampf(attitude_jump /
                        (2.0f * det->fusion.attitude_jump_threshold_rad), 0.25f, 1.0f);
        evidence_count++;
    }
    det->report.confidence = evidence_count > 0u
        ? clampf(score / (float)evidence_count + 0.12f * (float)(evidence_count - 1u),
                 0.0f, 1.0f)
        : 0.0f;

    if (!det->armed || det->refractory_remaining > 0.0f || det->triggered) {
        det->report.state = det->triggered ? CONFIRMED_IMPACT : NO_IMPACT;
    } else if (evidence_count > 0u) {
        det->report.state = POSSIBLE_IMPACT;
        det->candidate_time += step;
        if (det->confirm_count < 255u) {
            det->confirm_count++;
        }
        if (det->confirm_count >= det->cfg.confirm_samples &&
            det->candidate_time >= det->fusion.debounce_s &&
            det->report.confidence >= det->fusion.minimum_confidence) {
            det->triggered = 1u;
            det->report.state = CONFIRMED_IMPACT;
            det->report.rising_edge = 1u;
            det->refractory_remaining = det->fusion.refractory_s;
        }
    } else {
        det->report.state = NO_IMPACT;
        det->candidate_time = 0.0f;
        det->confirm_count = 0u;
    }

    det->previous_velocity = velocity;
    det->previous_attitude = quat_normalize(attitude);
    det->previous_timestamp_ms = sample->timestamp_ms;
    det->have_previous_motion = 1u;
    return det->report;
}

uint8_t impact_detector_update(ImpactDetector *det, const ImuSample *sample)
{
    float dt = 0.01f;
    ImpactReport report;
    if (det->previous_timestamp_ms > 0u && sample->timestamp_ms > det->previous_timestamp_ms) {
        dt = (float)(sample->timestamp_ms - det->previous_timestamp_ms) * 0.001f;
    }
    report = impact_detector_update_fused(det, sample, det->previous_velocity,
                                          det->previous_attitude, dt);
    return report.rising_edge;
}

const char *impact_state_name(ImpactState state)
{
    switch (state) {
    case NO_IMPACT:        return "NO_IMPACT";
    case POSSIBLE_IMPACT:  return "POSSIBLE_IMPACT";
    case CONFIRMED_IMPACT: return "CONFIRMED_IMPACT";
    default:               return "?";
    }
}
