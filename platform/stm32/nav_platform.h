/*
 * nav_platform.h - STM32 板级接口契约
 *
 * core/ 与 perception/ 的全部算法都是纯函数（输入结构体→输出结构体），
 * 与硬件解耦。移植到 STM32 时，板级代码只需实现本文件的接口：
 * 传感器读取、时间、飞控指令下发、日志。
 *
 * 约束（General_Plan 第 12 节）：不允许 malloc/异常/RTTI/STL 动态容器；
 * 所有实现必须静态内存、有界执行时间。
 *
 * 返回约定：0 = 成功，<0 = 本帧无效（传感器未就绪/通信失败）。
 */
#ifndef NAV_PLATFORM_H
#define NAV_PLATFORM_H

#include <stdint.h>
#include "state_estimator.h"    /* ImuSample / OdomSample 间接需要 */
#include "impact_detector.h"    /* ImuSample */
#include "camera.h"             /* PixelObs */
#include "vision_frontend.h"    /* FlowFrame */
#include "pos_controller.h"     /* CtrlOutput */

#ifdef __cplusplus
extern "C" {
#endif

/* 系统时间（ms，单调） */
uint32_t nav_time_ms(void);

/*
 * IMU：机体系比力 (m/s^2) + 陀螺 (rad/s)。
 * 低层以 >=500Hz 采样供姿态环；导航层在此接口按导航频率抽取/平均后的样本。
 */
int nav_imu_read(ImuSample *out);

/* ToF 测距：沿机体 -z 的距离 (m)。无效时返回 <0 */
int nav_tof_read(float *height_m);

/*
 * 光流帧：推荐由外部光流模组（如 PMW3901 类）输出特征/块匹配结果，
 * 或在片上前端提取。id 关联由模组/前端维护。
 * 无新帧时返回 <0（此时应保持上一帧有效性语义：valid=0）。
 */
int nav_flow_read(FlowFrame *out);

/* 前视相机目标检测（像素观测）；未检出/无新帧返回 <0 */
int nav_camera_target_read(PixelObs *out);

/* 下视相机基座 marker 检测（像素观测）；未检出/无新帧返回 <0 */
int nav_camera_home_read(PixelObs *out);

/*
 * 导航层输出：期望加速度/偏航角速度指令 → 低层飞控。
 * 低层飞控（姿态/角速度环、混控、电调）具有系统最高优先级，
 * 即使导航层完全失效也必须保持姿态稳定（Plan 第 9 节）。
 */
void nav_fcu_send(const CtrlOutput *cmd);

/* 解锁/断油控制（由 Mission FSM 状态驱动） */
void nav_fcu_set_armed(uint8_t armed);

/* 日志（SWO/UART/黑匣子，低频调用） */
void nav_log(const char *msg);

#ifdef __cplusplus
}
#endif

#endif /* NAV_PLATFORM_H */
