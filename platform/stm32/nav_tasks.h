/*
 * nav_tasks.h - STM32 固件任务装配（参考实现）
 *
 * NavApp 静态持有全部导航模块实例与配置，nav_app_step 为导航主循环
 * （建议 100~200 Hz，与 host 仿真 main.c 保持相同的模块调用顺序）。
 *
 * 低层姿态/角速度环不在本仓库范围（由既有固件或后续开发提供），
 * 本层只输出 CtrlOutput。
 */
#ifndef NAV_TASKS_H
#define NAV_TASKS_H

#include <stdint.h>
#include "state_estimator.h"
#include "impact_detector.h"
#include "safety_monitor.h"
#include "target_tracker.h"
#include "home_detector.h"
#include "vision_frontend.h"
#include "mission_fsm.h"
#include "pos_controller.h"
#include "trajectory.h"
#include "camera.h"
#include "guidance.h"      /* TerminalParams / HomeParams */

#ifdef __cplusplus
extern "C" {
#endif

/* 固件侧全部配置（对应仿真的 Scenario，但不含仿真专用字段） */
typedef struct {
    MissionConfig       mission;
    EstimatorConfig     estimator;
    ImpactDetectorConfig impact;
    SafetyConfig        safety;
    PosCtrlParams       ctrl;
    TerminalParams      terminal;
    HomeParams          home_guidance;
    FlowConfig          flow;
    CameraModel         cam_forward;
    CameraModel         cam_down;
    float               target_size_m;
    float               marker_size_m;
    float               recovery_damping;
    Vec3f               home_pos;
    WaypointQueue       outbound_route;
    WaypointQueue       search_route;
} NavAppConfig;

typedef struct {
    NavAppConfig   cfg;
    StateEstimator estimator;
    ImpactDetector impact_det;
    TargetTracker  tracker;
    HomeDetector   home_det;
    SafetyMonitor  safety;
    MissionFsm     fsm;
    VisionFrontend vf;
    Trajectory     traj;
    float          traj_t;
    uint8_t        traj_active;

    MissionState   prev_state;
    float          last_impact_t;
    float          last_relocalize_t;
    MissionOutput  mout;
} NavApp;

void nav_app_init(NavApp *app, const NavAppConfig *cfg);

/*
 * 导航主循环一步（建议 100~200 Hz 周期调用）：
 * 读取板级传感器 → 估计 → 检测 → FSM → 制导 → 控制 → nav_fcu_send。
 */
void nav_app_step(NavApp *app, float dt);

#ifdef __cplusplus
}
#endif

#endif /* NAV_TASKS_H */
