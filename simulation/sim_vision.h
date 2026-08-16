/*
 * sim_vision.h - 仿真视觉前端输入：地面特征点散射 + 帧生成
 *
 * 场地地面（z=0）随机散布特征点（已知场地假设，仅仿真侧使用）。
 * 每帧将可见特征点投影进下视相机，加像素噪声，保持 id 关联。
 * 光流算法本身在 perception/vision_frontend（机载算法）。
 */
#ifndef SIM_VISION_H
#define SIM_VISION_H

#include <stdint.h>
#include "nav_math.h"
#include "camera.h"
#include "vision_frontend.h"
#include "sim_dynamics.h"
#include "sim_sensors.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SIM_VISION_MAX_POINTS 4096u

typedef struct {
    Vec3f    points[SIM_VISION_MAX_POINTS];  /* 地面特征点（z=0） */
    uint16_t count;
} SimVisionWorld;

/* 在 [-area/2, area/2]^2 地面散布 count 个特征点（seed 决定） */
void sim_vision_world_init(SimVisionWorld *w, uint32_t seed, float area, uint16_t count);

/* 生成一帧特征观测；vision_freeze=1 或无可特点时 count=0 */
void sim_vision_frame(SimSensors *sen, const SimVisionWorld *w,
                      const SimState *truth, const CameraModel *cam,
                      uint8_t vision_freeze, uint32_t t_ms, FlowFrame *out);

#ifdef __cplusplus
}
#endif

#endif /* SIM_VISION_H */
