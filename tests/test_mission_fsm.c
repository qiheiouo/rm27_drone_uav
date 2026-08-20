/*
 * test_mission_fsm.c - 任务状态机关键转移单元测试
 *
 * 用合成输入驱动 FSM 走完关键链路，验证：
 *   正常链路转移、目标丢失回退、估计器丢失悬停恢复、超时返航。
 */
#include <stdio.h>
#include "mission_fsm.h"
#include "scenario.h"

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

static MissionInput make_input(const Scenario *sc)
{
    MissionInput in = {0};
    in.nav.pos = sc->home_pos;
    in.nav.vel = vec3_zero();
    in.nav.yaw = 0.0f;
    in.nav.yaw_rate = 0.0f;
    in.nav.att = quat_identity();
    in.nav.angular_velocity = vec3_zero();
    in.nav.linear_acceleration = vec3_zero();
    in.nav.validity = NAV_VALID_VALID;
    in.nav.quality = 1.0f;
    in.nav.mode = EST_MODE_INS;
    in.nav.status = EST_TRACKING;
    in.nav.timestamp_ms = 0u;
    in.target.visible = 0u;
    in.target.status = TARGET_TRACK_LOST;
    in.target.rel_pos = vec3_zero();
    in.target.range = 1e9f;
    in.target.time_since_update = 1e9f;
    in.target.confidence = 0.0f;
    in.home.visible = 0u;
    in.home.confidence = 0.0f;
    in.home.rel_pos = vec3_zero();
    in.home.time_since_update = 1e9f;
    in.home.relative_yaw = 0.0f;
    in.impact_detected = 0u;
    in.start_command = 1u;
    in.time_exceeded = 0u;
    in.geofence_violation = 0u;
    in.dt = 0.01f;
    return in;
}

/* 以当前输入跑 n 个 tick，返回最后一次输出 */
static void run_ticks(MissionFsm *fsm, MissionInput *in, MissionOutput *out, int n)
{
    int i;
    for (i = 0; i < n; i++) {
        mission_fsm_update(fsm, in, out);
    }
}

int main(void)
{
    Scenario sc;
    scenario_default(&sc);

    MissionFsm fsm;
    mission_fsm_init(&fsm, &sc.mission, sc.home_pos,
                     &sc.outbound_route, &sc.search_route);

    MissionInput in = make_input(&sc);
    MissionOutput out;

    /* BOOT → SELF_CHECK（第一个 tick 即转移） */
    run_ticks(&fsm, &in, &out, 1);
    CHECK(out.state == MS_SELF_CHECK);

    /* SELF_CHECK → DOCKED */
    run_ticks(&fsm, &in, &out, (int)(sc.mission.self_check_s / in.dt) + 2);
    CHECK(out.state == MS_DOCKED);

    /* DOCKED → TAKEOFF（start_command + 起飞延迟） */
    run_ticks(&fsm, &in, &out, (int)(sc.mission.docked_launch_delay_s / in.dt) + 2);
    CHECK(out.state == MS_TAKEOFF);
    CHECK(out.guidance == GM_TAKEOFF);

    /* TAKEOFF → OUTBOUND：报告到达起飞高度 */
    in.nav.pos = vec3(sc.home_pos.x, sc.home_pos.y, sc.mission.takeoff_alt_m);
    run_ticks(&fsm, &in, &out, 2);
    CHECK(out.state == MS_OUTBOUND);

    /* OUTBOUND：依次经过所有出航航点 → SEARCH */
    {
        uint8_t wi;
        for (wi = 0u; wi < sc.outbound_route.count; wi++) {
            in.nav.pos = sc.outbound_route.items[wi].pos;
            run_ticks(&fsm, &in, &out, 2);
        }
    }
    run_ticks(&fsm, &in, &out, 2);
    CHECK(out.state == MS_SEARCH);

    /* SEARCH → TARGET_TRACK：目标持续可见 */
    in.target.visible = 1u;
    in.target.status = TARGET_TRACK_TRACKING;
    in.target.confidence = 1.0f;
    in.target.rel_pos = vec3(2.5f, 0.0f, 0.0f);
    in.target.range = 2.5f;
    run_ticks(&fsm, &in, &out, (int)(sc.mission.target_confirm_s / in.dt) + 2);
    CHECK(out.state == MS_TARGET_TRACK);
    CHECK(out.guidance == GM_TERMINAL);

    /* TARGET_TRACK → TERMINAL：距离小于终端门限 */
    in.target.range = sc.mission.terminal_range_m - 0.1f;
    run_ticks(&fsm, &in, &out, 2);
    CHECK(out.state == MS_TERMINAL);

    /* TERMINAL → IMPACT → RECOVERY */
    in.impact_detected = 1u;
    run_ticks(&fsm, &in, &out, 1);
    CHECK(out.state == MS_IMPACT);
    in.impact_detected = 0u;
    in.target.visible = 0u;
    run_ticks(&fsm, &in, &out, 1);
    CHECK(out.state == MS_RECOVERY);
    CHECK(out.guidance == GM_RECOVERY);

    /* RECOVERY → BREAKAWAY：稳定时间到且估计器健康 */
    run_ticks(&fsm, &in, &out, (int)(sc.mission.recovery_hold_s / in.dt) + 2);
    CHECK(out.state == MS_BREAKAWAY);

    /* BREAKAWAY → RETURN_HOME：到达 breakaway 目标 */
    in.nav.pos = fsm.breakaway_target;
    run_ticks(&fsm, &in, &out, 2);
    CHECK(out.state == MS_RETURN_HOME);

    /* RETURN_HOME → HOME_SEARCH：进入基座区域 */
    in.nav.pos = vec3(sc.home_pos.x, sc.home_pos.y, sc.mission.takeoff_alt_m);
    run_ticks(&fsm, &in, &out, 2);
    CHECK(out.state == MS_HOME_SEARCH);
    CHECK(out.guidance == GM_HOME_SERVO);

    /* HOME_SEARCH → DOCKING：marker 可见 */
    in.home.visible = 1u;
    in.home.rel_pos = vec3(0.0f, 0.0f, -1.0f);
    in.home.time_since_update = 0.0f;
    run_ticks(&fsm, &in, &out, 2);
    CHECK(out.state == MS_HOMING);

    in.nav.pos.z = sc.mission.dock_blind_land_alt_m - 0.01f;
    run_ticks(&fsm, &in, &out, 4);
    CHECK(out.state == MS_DOCKING);

    /* DOCKING → DOCKED：对准且高度足够低 */
    in.nav.pos = vec3(sc.home_pos.x, sc.home_pos.y, sc.mission.dock_alt_m - 0.01f);
    in.home.rel_pos = vec3(0.01f, 0.01f, -0.05f);
    in.dock_contact = 1u;
    in.wireless_charge_ready = 1u;
    run_ticks(&fsm, &in, &out, 8);
    CHECK(out.state == MS_DOCKED);
    CHECK(out.mission_complete == 1u);

    printf("test_mission_fsm: nominal chain PASS\n");

    /* ---------- 异常路径 1：目标丢失回 SEARCH ---------- */
    {
        MissionFsm f2;
        mission_fsm_init(&f2, &sc.mission, sc.home_pos,
                         &sc.outbound_route, &sc.search_route);
        MissionInput in2 = make_input(&sc);
        MissionOutput o2;

        run_ticks(&f2, &in2, &o2, 1);
        run_ticks(&f2, &in2, &o2, (int)(sc.mission.self_check_s / in2.dt) + 2);
        run_ticks(&f2, &in2, &o2, (int)(sc.mission.docked_launch_delay_s / in2.dt) + 2);
        in2.nav.pos = vec3(sc.home_pos.x, sc.home_pos.y, sc.mission.takeoff_alt_m);
        run_ticks(&f2, &in2, &o2, 2);
        {
            uint8_t wi;
            for (wi = 0u; wi < sc.outbound_route.count; wi++) {
                in2.nav.pos = sc.outbound_route.items[wi].pos;
                run_ticks(&f2, &in2, &o2, 2);
            }
        }
        run_ticks(&f2, &in2, &o2, 2);
        CHECK(o2.state == MS_SEARCH);

        in2.target.visible = 1u;
        in2.target.status = TARGET_TRACK_TRACKING;
        in2.target.confidence = 1.0f;
        in2.target.range = 2.5f;
        run_ticks(&f2, &in2, &o2, (int)(sc.mission.target_confirm_s / in2.dt) + 2);
        CHECK(o2.state == MS_TARGET_TRACK);

        in2.target.visible = 0u;
        run_ticks(&f2, &in2, &o2, (int)(sc.mission.target_lost_timeout_s / in2.dt) + 2);
        CHECK(o2.state == MS_SEARCH);
        printf("test_mission_fsm: target-lost fallback PASS\n");
    }

    /* ---------- 异常路径 2：估计器丢失悬停 → 恢复 / 超时降落 ---------- */
    {
        MissionFsm f3;
        mission_fsm_init(&f3, &sc.mission, sc.home_pos,
                         &sc.outbound_route, &sc.search_route);
        MissionInput in3 = make_input(&sc);
        MissionOutput o3;

        run_ticks(&f3, &in3, &o3, 1);
        run_ticks(&f3, &in3, &o3, (int)(sc.mission.self_check_s / in3.dt) + 2);
        run_ticks(&f3, &in3, &o3, (int)(sc.mission.docked_launch_delay_s / in3.dt) + 2);
        in3.nav.pos = vec3(sc.home_pos.x, sc.home_pos.y, sc.mission.takeoff_alt_m);
        run_ticks(&f3, &in3, &o3, 2);
        CHECK(o3.state == MS_OUTBOUND);

        in3.nav.status = EST_LOST;
        run_ticks(&f3, &in3, &o3, 2);
        CHECK(o3.state == MS_ESTIMATOR_LOST);
        CHECK(o3.guidance == GM_HOLD);

        in3.nav.status = EST_TRACKING;
        run_ticks(&f3, &in3, &o3, 2);
        CHECK(o3.state == MS_OUTBOUND);   /* 恢复到原状态 */

        in3.nav.status = EST_LOST;
        run_ticks(&f3, &in3, &o3, 2);
        run_ticks(&f3, &in3, &o3, (int)(sc.mission.estimator_lost_timeout_s / in3.dt) + 2);
        CHECK(o3.state == MS_EMERGENCY_LAND);
        printf("test_mission_fsm: estimator-lost handling PASS\n");
    }

    /* ---------- 异常路径 3：任务超时直接返航 ---------- */
    {
        MissionFsm f4;
        mission_fsm_init(&f4, &sc.mission, sc.home_pos,
                         &sc.outbound_route, &sc.search_route);
        MissionInput in4 = make_input(&sc);
        MissionOutput o4;

        run_ticks(&f4, &in4, &o4, 1);
        run_ticks(&f4, &in4, &o4, (int)(sc.mission.self_check_s / in4.dt) + 2);
        run_ticks(&f4, &in4, &o4, (int)(sc.mission.docked_launch_delay_s / in4.dt) + 2);
        in4.nav.pos = vec3(sc.home_pos.x, sc.home_pos.y, sc.mission.takeoff_alt_m);
        run_ticks(&f4, &in4, &o4, 2);
        CHECK(o4.state == MS_OUTBOUND);

        /* 无人机在远离 home 处超时 → 直接返航（且不应立刻进入 HOME_SEARCH） */
        in4.nav.pos = vec3(3.0f, 0.0f, sc.mission.takeoff_alt_m);
        in4.time_exceeded = 1u;
        run_ticks(&f4, &in4, &o4, 2);
        CHECK(o4.state == MS_RETURN_HOME);
        printf("test_mission_fsm: low-battery abort PASS\n");
    }

    if (failures == 0) {
        printf("test_mission_fsm: PASS\n");
        return 0;
    }
    printf("test_mission_fsm: %d FAILURES\n", failures);
    return 1;
}
