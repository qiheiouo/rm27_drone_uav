/*
 * scenario.h - 仿真场景与全部参数配置
 *
 * 已知场地假设（Plan 第 11 节）：arena 几何、目标大致区域、
 * 基座位置均为先验；第一版不做在线建图。
 */
#ifndef SCENARIO_H
#define SCENARIO_H

#include "nav_math.h"
#include "waypoint.h"
#include "mission_fsm.h"
#include "state_estimator.h"
#include "impact_detector.h"
#include "safety_monitor.h"
#include "pos_controller.h"
#include "guidance.h"
#include "camera.h"
#include "vision_frontend.h"
#include "sim_dynamics.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* 场景真值 */
    Vec3f home_pos;
    Vec3f target_pos;
    WaypointQueue outbound_route;
    WaypointQueue search_route;

    /* 模块配置 */
    MissionConfig     mission;
    EstimatorConfig   estimator;
    ImpactDetectorConfig impact;
    SafetyConfig      safety;
    PosCtrlParams     ctrl;
    SimParams         dynamics;
    TerminalParams    terminal;
    HomeParams        home_guidance;
    float             recovery_damping;

    /* 相机（第二阶段：像素级检测链路） */
    CameraModel       cam_forward;   /* 前视：目标检测 */
    CameraModel       cam_down;      /* 下视：基座 marker + 光流 */
    float             target_size_m; /* 目标真实尺寸（检测算法已知） */
    float             marker_size_m; /* 基座 marker 真实尺寸 */

    /* VO 前端（第三阶段：光流里程计） */
    uint8_t           use_flow_vo;   /* 1 = 光流前端；0 = 仿真 VO（对照） */
    FlowConfig        flow;
    float             feature_area_m;    /* 地面特征点散布范围 */
    uint16_t          feature_count;     /* 地面特征点数量 */

    /* 仿真控制 */
    float dt;
    float sim_max_time_s;
    float impact_range_m;    /* TERMINAL 中距目标小于该值时发生接触 */
    float vision_freeze_s;   /* 撞击后视觉冻结时长 */
    uint32_t seed;
} Scenario;

/* 默认场景：基座在原点，搜索区约 4 m 外，目标在搜索区内 */
void scenario_default(Scenario *sc);

#ifdef __cplusplus
}
#endif

#endif /* SCENARIO_H */
