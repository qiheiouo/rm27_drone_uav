/*
 * mission_fsm.h - 任务状态机（Plan 第 14 节）
 *
 * 正常链路：
 *   BOOT → SELF_CHECK → DOCKED → TAKEOFF → OUTBOUND → SEARCH
 *   → TARGET_TRACK → TERMINAL → IMPACT → RECOVERY → BREAKAWAY
 *   → RETURN_HOME → HOME_SEARCH → DOCKING → DOCKED(完成)
 *
 * 异常处理：
 *   - 估计器 LOST（非撞击恢复期）→ ESTIMATOR_LOST → 恢复后继续 / 超时 EMERGENCY_LAND
 *   - 目标丢失 → 回 SEARCH（即 Plan 中的 TARGET_LOST 行为）
 *   - 超时/低电量 → 立即 RETURN_HOME（LOW_BATTERY 行为）
 *   - 地理围栏 violation → EMERGENCY_STABILIZE → EMERGENCY_LAND
 */
#ifndef MISSION_FSM_H
#define MISSION_FSM_H

#include <stdint.h>
#include "nav_math.h"
#include "state_estimator.h"
#include "target_tracker.h"
#include "home_detector.h"
#include "waypoint.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MS_BOOT = 0,
    MS_SELF_CHECK,
    MS_DOCKED,
    MS_TAKEOFF,
    MS_OUTBOUND,
    MS_SEARCH,
    MS_TARGET_TRACK,
    MS_TERMINAL,
    MS_IMPACT,
    MS_RECOVERY,
    MS_BREAKAWAY,
    MS_RETURN_HOME,
    MS_HOME_SEARCH,
    MS_DOCKING,
    MS_ESTIMATOR_LOST,
    MS_EMERGENCY_STABILIZE,
    MS_EMERGENCY_LAND
} MissionState;

/* FSM → 制导层 的模式选择 */
typedef enum {
    GM_NONE = 0,        /* 电机怠速/断电 */
    GM_HOLD,            /* 位置保持 */
    GM_TAKEOFF,         /* 起飞到指定高度 */
    GM_WAYPOINT,        /* 跟踪 current_waypoint */
    GM_TERMINAL,        /* 目标相对视觉伺服 */
    GM_RECOVERY,        /* 撞击后速度阻尼 */
    GM_HOME_SERVO,      /* 基座 marker 视觉伺服（搜索 + 停靠） */
    GM_LAND             /* 紧急垂直降落 */
} GuidanceMode;

typedef struct {
    float takeoff_alt_m;            /* 起飞悬停高度 */
    float waypoint_tol_m;           /* 航点到达容差 */
    float cruise_speed_mps;         /* 巡航速度 */
    float target_confirm_s;         /* 目标连续可见确认时间 */
    float terminal_range_m;         /* TARGET_TRACK → TERMINAL 的距离门限 */
    float target_lost_timeout_s;    /* 目标丢失超时 → 回 SEARCH */
    float recovery_hold_s;          /* RECOVERY 最短稳定时间 */
    float breakaway_height_m;       /* BREAKAWAY 爬升高度 */
    float home_region_tol_m;        /* 距 home 多少米内认为进入基座区域 */
    float home_search_alt_m;        /* HOME_SEARCH 的搜索高度 */
    float dock_alt_m;               /* 低于该高度且横向对准 → 判定停靠成功 */
    float dock_lateral_tol_m;       /* 停靠横向容差（视觉对准） */
    float dock_capture_tol_m;       /* 机械捕获容差（基座停靠结构，marker 暂失也允许） */
    float dock_capture_max_alt_m;   /* 机械捕获判定的最大估计高度（防止空中误捕获） */
    float dock_land_vel_max;        /* 着陆判定速度上限 (m/s) */
    float dock_blind_land_alt_m;    /* 低于该高度 marker 暂失时继续盲降而非重新搜索 */
    float home_lost_timeout_s;      /* 停靠中 marker 丢失超时 → 回 HOME_SEARCH */
    float estimator_lost_timeout_s; /* 估计器丢失最长等待 → 紧急降落 */
    float self_check_s;
    float docked_launch_delay_s;    /* 自动起飞前等待 */
} MissionConfig;

typedef struct {
    NavState   nav;
    TargetTrack target;
    HomeTrack  home;
    uint8_t    impact_detected;     /* 撞击脉冲（上升沿） */
    uint8_t    start_command;
    uint8_t    time_exceeded;       /* safety: 任务超时/低电量 */
    uint8_t    geofence_violation;
    float      dt;
} MissionInput;

typedef struct {
    MissionState state;
    GuidanceMode guidance;
    Waypoint     current_waypoint;  /* GM_WAYPOINT 有效 */
    Vec3f        hold_pos;          /* GM_TAKEOFF / GM_HOLD 有效 */
    float        home_search_alt;   /* GM_HOME_SERVO 有效 */
    uint8_t      state_changed;     /* 本 tick 发生了状态切换（用于日志） */
    uint8_t      mission_complete;  /* 已停靠，任务成功 */
    uint8_t      mission_failed;    /* 紧急降落等终止 */
} MissionOutput;

typedef struct {
    MissionConfig cfg;
    Vec3f      home_pos;
    MissionState state;
    MissionState resume_state;      /* ESTIMATOR_LOST 恢复后回到的状态 */
    float      state_time;
    float      target_visible_time;
    float      target_lost_time;
    float      estimator_lost_time;
    float      breakaway_speed;
    WaypointQueue outbound;
    WaypointQueue search;
    Vec3f      breakaway_target;
    uint8_t    mission_complete;
    uint8_t    mission_failed;
} MissionFsm;

void mission_fsm_init(MissionFsm *fsm,
                      const MissionConfig *cfg,
                      Vec3f home_pos,
                      const WaypointQueue *outbound,
                      const WaypointQueue *search);

void mission_fsm_update(MissionFsm *fsm, const MissionInput *in, MissionOutput *out);

const char *mission_state_name(MissionState s);

#ifdef __cplusplus
}
#endif

#endif /* MISSION_FSM_H */
