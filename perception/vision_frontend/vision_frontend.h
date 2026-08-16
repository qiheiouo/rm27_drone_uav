/*
 * vision_frontend.h - 视觉前端：下视光流里程计（第三阶段，真实算法）
 *
 * 职责：地面特征点光流 + 陀螺 → 机体系速度 → 导航系 OdomSample。
 *
 * 原理（MCU 可运行，固定容量，无动态内存）：
 *   1. 帧间特征按 id 关联得到光流 (du, dv)
 *   2. 旋转补偿：du_rot = fx( xy·ωx - (1+x²)·ωy + y·ωz )（标准光流方程，
 *      陀螺经安装矩阵变换到相机系），从实测光流中扣除
 *   3. 平移最小二乘：du = fx(-Vx + x·Vz)/h，dv = fy(-Vy + y·Vz)/h，
 *      h = 离地高度（地面平面假设），3 未知量正规方程闭式求解
 *   4. 残差剔除外点后再解一次；速度经姿态旋转到导航系并积分出位置
 *
 * 输出 OdomSample 直接对接 state_estimator 的 VO 校正通道。
 * 漂移不再由仿真注入，而是从像素噪声/高度误差中自然涌现。
 */
#ifndef VISION_FRONTEND_H
#define VISION_FRONTEND_H

#include <stdint.h>
#include "nav_math.h"
#include "state_estimator.h"   /* OdomSample */
#include "camera.h"            /* CameraModel / 安装矩阵 */

#ifdef __cplusplus
extern "C" {
#endif

#define VF_MAX_FEATURES 48u

/* 单帧特征观测（id 由传感器/仿真器保持关联） */
typedef struct {
    uint16_t id;
    float    u, v;
} FlowFeature;

typedef struct {
    FlowFeature feats[VF_MAX_FEATURES];
    uint8_t     count;
    uint32_t    timestamp_ms;
} FlowFrame;

typedef struct {
    float   min_height, max_height; /* 有效离地高度范围 (m) */
    uint8_t min_features;           /* 最少匹配特征数 */
    float   outlier_residual_px;    /* 外点剔除残差门限 (px) */
} FlowConfig;

typedef struct {
    FlowConfig cfg;
    CameraModel cam;                /* 下视相机（光流相机） */

    /* 上一帧 */
    uint16_t prev_id[VF_MAX_FEATURES];
    float    prev_u[VF_MAX_FEATURES];
    float    prev_v[VF_MAX_FEATURES];
    uint8_t  prev_count;
    uint8_t  has_prev;

    /* VO 坐标系积分状态（起飞点为原点） */
    Vec3f    vo_pos;
    float    vo_yaw;

    /* 最近一次解算诊断 */
    uint8_t  last_matched;
    float    last_mean_residual;
} VisionFrontend;

void vf_init(VisionFrontend *vf, const FlowConfig *cfg, const CameraModel *cam);
void vf_reset_track(VisionFrontend *vf);   /* 撞击/模糊后调用，丢弃上一帧 */
/* 重定位时对齐 VO 坐标系到绝对参考（防止 VO 系与估计系持续发散） */
void vf_set_pose(VisionFrontend *vf, Vec3f pos, float yaw);

/*
 * 每帧调用。gyro_body：机体系陀螺 (rad/s)；att：当前姿态估计；
 * height：离地高度（m，来自估计器 z）；dt：帧间隔。
 * out->valid = 1 时 pos/vel/yaw 有效。
 */
void vf_update(VisionFrontend *vf, const FlowFrame *frame,
               Vec3f gyro_body, Quatf att, float height, float dt,
               OdomSample *out);

#ifdef __cplusplus
}
#endif

#endif /* VISION_FRONTEND_H */
