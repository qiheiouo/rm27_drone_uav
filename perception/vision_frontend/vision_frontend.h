/*
 * vision_frontend.h - 视觉前端接口占位（第一阶段不实现）
 *
 * 未来职责：特征提取/跟踪、光流、关键帧管理，
 * 向 state_estimator 提供视觉量测（OdomSample 的 valid/质量来源）。
 * 第一阶段传感器为仿真真值，此文件仅固定数据流位置与质量度量结构，
 * 保证后续接入真实相机时上层接口不变。
 */
#ifndef VISION_FRONTEND_H
#define VISION_FRONTEND_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t tracked_features;  /* 当前跟踪到的特征点数 */
    float    mean_parallax;     /* 平均视差 (rad)，用于判断可三角化程度 */
    uint8_t  quality;           /* 0..100 综合质量分，低于阈值时估计器应降权 */
} VisionFrontendStatus;

#ifdef __cplusplus
}
#endif

#endif /* VISION_FRONTEND_H */
