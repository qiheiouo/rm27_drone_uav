#include "impact_detector.h"

void impact_detector_init(ImpactDetector *det, const ImpactDetectorConfig *cfg)
{
    det->cfg = *cfg;
    det->triggered = 0u;
    det->confirm_count = 0u;
}

void impact_detector_reset(ImpactDetector *det)
{
    det->triggered = 0u;
    det->confirm_count = 0u;
}

uint8_t impact_detector_update(ImpactDetector *det, const ImuSample *sample)
{
    uint8_t edge = 0u;
    /* 加速度通道：比力模长偏离 1g（悬停/平稳飞行基线）判定，
     * 正常机动偏离 <~7 m/s^2，撞击脉冲通常 >100 m/s^2 */
    float accel_dev = fabsf(vec3_norm(sample->accel) - NAV_GRAVITY);
    uint8_t spike = (accel_dev >= det->cfg.accel_spike_threshold ||
                     vec3_norm(sample->gyro) >= det->cfg.gyro_spike_threshold) ? 1u : 0u;

    if (spike) {
        if (det->confirm_count < 255u) {
            det->confirm_count++;
        }
    } else {
        det->confirm_count = 0u;
    }

    if (!det->triggered && det->confirm_count >= det->cfg.confirm_samples) {
        det->triggered = 1u;
        edge = 1u;
    }
    return edge;
}
