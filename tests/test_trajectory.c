/* test_trajectory.c - 多项式轨迹（MINCO-lite）单元测试 */
#include <stdio.h>
#include <math.h>
#include "trajectory.h"

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

int main(void)
{
    /* ---- 1. 单段边界条件：起止位置/速度精确满足 ---- */
    {
        Trajectory traj;
        Vec3f p0 = vec3(0.0f, 0.0f, 1.0f);
        Vec3f v0 = vec3(0.5f, 0.0f, 0.0f);
        Vec3f p1 = vec3(3.0f, 2.0f, 1.2f);
        traj_build_single(&traj, p0, v0, p1, 1.5f, 1.6f);

        CHECK(traj.count == 1u);
        CHECK(traj.duration > 0.3f);

        Vec3f pos, vel, acc;
        traj_evaluate(&traj, 0.0f, &pos, &vel, &acc);
        CHECK(vec3_dist(pos, p0) < 1e-4f);
        CHECK(vec3_dist(vel, v0) < 1e-4f);

        traj_evaluate(&traj, traj.duration, &pos, &vel, &acc);
        CHECK(vec3_dist(pos, p1) < 1e-3f);
        CHECK(vec3_norm(vel) < 1e-3f);          /* 终点停止 */
        CHECK(vec3_norm(acc) < 1e-2f);          /* 终点加速度≈0 */
        CHECK(traj_done(&traj, traj.duration + 0.1f) == 1u);
        CHECK(traj_done(&traj, traj.duration * 0.5f) == 0u);
        printf("test_trajectory: boundary conditions PASS (T=%.2f s)\n", traj.duration);
    }

    /* ---- 2. 峰值速度在时间分配预期内（min-jerk 峰值 ≈ 1.875*dist/T） ---- */
    {
        Trajectory traj;
        Vec3f p0 = vec3_zero();
        Vec3f p1 = vec3(4.0f, 0.0f, 0.0f);
        float speed = 1.5f;
        traj_build_single(&traj, p0, vec3_zero(), p1, speed, 1.6f);

        float vmax = 0.0f;
        for (float t = 0.0f; t <= traj.duration; t += 0.01f) {
            Vec3f pos, vel, acc;
            traj_evaluate(&traj, t, &pos, &vel, &acc);
            float v = vec3_norm(vel);
            if (v > vmax) { vmax = v; }
        }
        float expected = 1.875f * 4.0f / traj.duration;
        CHECK(fabsf(vmax - expected) < 0.05f * expected);
        CHECK(vmax < 1.25f * speed);
        printf("test_trajectory: peak velocity %.2f m/s (expected %.2f) PASS\n", vmax, expected);
    }

    /* ---- 3. 多段：段间位置/速度连续，经过所有航点 ---- */
    {
        WaypointQueue route;
        wq_init(&route);
        wq_push(&route, vec3(2.0f, 0.0f, 1.0f), 1.5f);
        wq_push(&route, vec3(2.0f, 2.0f, 1.0f), 1.5f);
        wq_push(&route, vec3(0.0f, 2.0f, 1.5f), 1.5f);

        Trajectory traj;
        traj_build(&traj, vec3_zero(), vec3_zero(), &route, 1.6f);
        CHECK(traj.count == 3u);

        /* 段间连续性：第 i 段末端 == 第 i+1 段起点 */
        float t = 0.0f;
        for (uint8_t i = 0u; i < traj.count; i++) {
            t += traj.segs[i].dur;
            Vec3f pos_end, vel_end, acc;
            traj_evaluate(&traj, t, &pos_end, &vel_end, &acc);
            CHECK(vec3_dist(pos_end, route.items[i].pos) < 1e-3f);
            CHECK(vec3_norm(vel_end) < 1e-3f);
        }
        /* 总时长 = 各段之和 */
        float sum = 0.0f;
        for (uint8_t i = 0u; i < traj.count; i++) { sum += traj.segs[i].dur; }
        CHECK(fabsf(sum - traj.duration) < 1e-4f);
        printf("test_trajectory: multi-segment continuity PASS\n");
    }

    /* ---- 4. 超范围 evaluate clamp 到终点 ---- */
    {
        Trajectory traj;
        Vec3f p1 = vec3(1.0f, 1.0f, 1.0f);
        traj_build_single(&traj, vec3_zero(), vec3_zero(), p1, 1.5f, 1.6f);
        Vec3f pos, vel, acc;
        traj_evaluate(&traj, traj.duration + 5.0f, &pos, &vel, &acc);
        CHECK(vec3_dist(pos, p1) < 1e-3f);
        CHECK(vec3_norm(vel) < 1e-3f);
        printf("test_trajectory: clamp PASS\n");
    }

    if (failures == 0) {
        printf("test_trajectory: PASS\n");
        return 0;
    }
    printf("test_trajectory: %d FAILURES\n", failures);
    return 1;
}
