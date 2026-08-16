/*
 * lowpass.h - 一阶低通滤波器（MCU 兼容，header-only）
 *
 * 用于感知输出的平滑（如目标相对位置），固定状态、无动态内存。
 */
#ifndef LOWPASS_H
#define LOWPASS_H

#include <stdint.h>
#include "nav_math.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float alpha;        /* 平滑系数，由截止频率与 dt 决定 */
    float y;            /* 当前输出 */
    uint8_t initialized;
} LowPass1;

static inline void lp_init(LowPass1 *f, float cutoff_hz, float dt)
{
    float rc = 1.0f / (NAV_2PI * cutoff_hz);
    f->alpha = dt / (rc + dt);
    f->y = 0.0f;
    f->initialized = 0u;
}

static inline float lp_update(LowPass1 *f, float x)
{
    if (!f->initialized) {
        f->y = x;
        f->initialized = 1u;
    } else {
        f->y += f->alpha * (x - f->y);
    }
    return f->y;
}

static inline void lp_reset(LowPass1 *f)
{
    f->y = 0.0f;
    f->initialized = 0u;
}

#ifdef __cplusplus
}
#endif

#endif /* LOWPASS_H */
