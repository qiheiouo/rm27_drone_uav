/* test_impact_detector.c - 撞击检测单元测试（比力偏离 1g 判定） */
#include <stdio.h>
#include "impact_detector.h"

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

static ImuSample make_sample(float ax, float ay, float az, float gz, uint32_t t)
{
    ImuSample s;
    s.accel = vec3(ax, ay, az);
    s.gyro = vec3(0.0f, 0.0f, gz);
    s.timestamp_ms = t;
    return s;
}

int main(void)
{
    ImpactDetectorConfig cfg;
    cfg.accel_spike_threshold = 12.0f;   /* 偏离 1g 的门限 */
    cfg.gyro_spike_threshold = 6.0f;
    cfg.confirm_samples = 2u;

    ImpactDetector det;
    impact_detector_init(&det, &cfg);

    /* 悬停/正常机动（比力约 1g，机动偏离 <7）不触发 */
    uint32_t t = 0u;
    int i;
    for (i = 0; i < 100; i++) {
        ImuSample normal = make_sample(3.0f, 2.0f, 9.5f, 0.5f, t);  /* |a|≈10.2, dev≈0.4 */
        CHECK(impact_detector_update(&det, &normal) == 0u);
        t += 10u;
    }
    CHECK(det.triggered == 0u);

    /* 单帧尖峰 + confirm_samples=2：第一帧不锁存，第二帧确认 */
    ImuSample spike = make_sample(120.0f, 50.0f, 30.0f, 0.5f, t);   /* dev >100 */
    CHECK(impact_detector_update(&det, &spike) == 0u);
    CHECK(det.triggered == 0u);
    t += 10u;
    spike.timestamp_ms = t;
    CHECK(impact_detector_update(&det, &spike) == 1u);   /* 上升沿 */
    CHECK(det.triggered == 1u);

    /* 锁存：后续帧不再产生上升沿 */
    ImuSample calm = make_sample(0.0f, 0.0f, 9.81f, 0.1f, t + 10u);
    CHECK(impact_detector_update(&det, &calm) == 0u);
    CHECK(det.triggered == 1u);

    /* 复位后可再次检测（gyro 通道） */
    impact_detector_reset(&det);
    CHECK(det.triggered == 0u);
    ImuSample gyro_spike = make_sample(0.0f, 0.0f, 9.81f, 9.0f, t + 20u);
    CHECK(impact_detector_update(&det, &gyro_spike) == 0u);
    gyro_spike.timestamp_ms = t + 30u;
    CHECK(impact_detector_update(&det, &gyro_spike) == 1u);

    if (failures == 0) {
        printf("test_impact_detector: PASS\n");
        return 0;
    }
    printf("test_impact_detector: %d FAILURES\n", failures);
    return 1;
}
