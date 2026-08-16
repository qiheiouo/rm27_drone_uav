#include "sim_vision.h"

void sim_vision_world_init(SimVisionWorld *w, uint32_t seed, float area, uint16_t count)
{
    uint32_t rng = seed ? (seed ^ 0x9E3779B9u) : 1u;
    uint16_t i;
    if (count > SIM_VISION_MAX_POINTS) {
        count = SIM_VISION_MAX_POINTS;
    }
    for (i = 0u; i < count; i++) {
        float ux, uy;
        rng = rng * 1664525u + 1013904223u;
        ux = ((float)(rng >> 8) / 8388608.0f) - 1.0f;
        rng = rng * 1664525u + 1013904223u;
        uy = ((float)(rng >> 8) / 8388608.0f) - 1.0f;
        w->points[i] = vec3(ux * area * 0.5f, uy * area * 0.5f, 0.0f);
    }
    w->count = count;
}

void sim_vision_frame(SimSensors *sen, const SimVisionWorld *w,
                      const SimState *truth, const CameraModel *cam,
                      uint8_t vision_freeze, uint32_t t_ms, FlowFrame *out)
{
    uint16_t i;

    out->count = 0u;
    out->timestamp_ms = t_ms;
    if (vision_freeze) {
        return;
    }

    for (i = 0u; i < w->count && out->count < VF_MAX_FEATURES; i++) {
        Vec3f rel_nav = vec3_sub(w->points[i], truth->pos);
        Vec3f rel_body = quat_rotate_inv(truth->att, rel_nav);
        float u, v, size_px;

        if (camera_project(cam, rel_body, 0.02f, &u, &v, &size_px)) {
            FlowFeature *f = &out->feats[out->count];
            f->id = i;
            /* 像素噪声 + 量化（模拟检测器定位精度） */
            sen->rng_state = sen->rng_state * 1664525u + 1013904223u;
            float n1 = ((float)(sen->rng_state >> 8) / 8388608.0f) - 1.0f;
            sen->rng_state = sen->rng_state * 1664525u + 1013904223u;
            float n2 = ((float)(sen->rng_state >> 8) / 8388608.0f) - 1.0f;
            f->u = u + n1 * sen->pixel_noise;
            f->v = v + n2 * sen->pixel_noise;
            out->count++;
        }
    }
}
