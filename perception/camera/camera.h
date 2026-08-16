/*
 * camera.h - 针孔相机模型 + 基于已知尺寸的目标重建（PnP-lite）
 *
 * 设计（Plan 第 7 节第二层）：目标/基座相对定位不依赖全局坐标。
 * 已知目标真实尺寸时，由像素位置 + 像素尺寸即可恢复相对位置：
 *   depth = fx * obj_size / size_px
 *   rel_cam = ((u-cx)/fx, (v-cy)/fy, 1) * depth
 *   rel_nav = R(att) * M^T * rel_cam
 *
 * camera_project 用于仿真成像；camera_reconstruct_nav 为机载算法（MCU 可运行）。
 *
 * 相机安装矩阵 M（3x3，行主序）：p_cam = M * p_body。
 * 前视相机（看机体 +x）：CAM_MOUNT_FORWARD
 * 下视相机（看机体 -z）：CAM_MOUNT_DOWN
 */
#ifndef CAMERA_H
#define CAMERA_H

#include <stdint.h>
#include "nav_math.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float fx, fy;      /* 焦距 (px) */
    float cx, cy;      /* 主点 (px) */
    float width, height;
    float m[9];        /* 机体 → 相机 旋转（行主序 3x3） */
} CameraModel;

/* 像素观测（检测器输入） */
typedef struct {
    uint8_t  visible;
    float    u, v;     /* 目标中心像素 */
    float    size_px;  /* 目标在图像中的表观尺寸 (px) */
    uint32_t timestamp_ms;
} PixelObs;

extern const float CAM_MOUNT_FORWARD[9];
extern const float CAM_MOUNT_DOWN[9];

void camera_init(CameraModel *cam, float fx, float fy, float cx, float cy,
                 float width, float height, const float mount[9]);

/* 仿真成像：rel_body 为真值相对位置（机体系）。visible 由视锥/图像边界决定。 */
uint8_t camera_project(const CameraModel *cam, Vec3f rel_body, float obj_size,
                       float *u, float *v, float *size_px);

/* 机载重建：像素观测 + 姿态 → 导航系相对位置 */
Vec3f camera_reconstruct_nav(const CameraModel *cam, float u, float v,
                             float size_px, float obj_size, Quatf att);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_H */
