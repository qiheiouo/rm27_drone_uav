/* test_camera.c - 相机投影/重建往返与视锥剔除测试 */
#include <stdio.h>
#include <math.h>
#include "camera.h"

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

int main(void)
{
    CameraModel fwd, down;
    camera_init(&fwd, 180.0f, 180.0f, 160.0f, 120.0f, 320.0f, 240.0f, CAM_MOUNT_FORWARD);
    camera_init(&down, 180.0f, 180.0f, 160.0f, 120.0f, 320.0f, 240.0f, CAM_MOUNT_DOWN);

    /* ---- 前视：正前方目标投影到主点 ---- */
    float u, v, s;
    CHECK(camera_project(&fwd, vec3(2.0f, 0.0f, 0.0f), 0.3f, &u, &v, &s) == 1u);
    CHECK(fabsf(u - 160.0f) < 1e-3f && fabsf(v - 120.0f) < 1e-3f);
    CHECK(fabsf(s - 180.0f * 0.3f / 2.0f) < 1e-3f);

    /* 相机后方 → 不可见 */
    CHECK(camera_project(&fwd, vec3(-1.0f, 0.0f, 0.0f), 0.3f, &u, &v, &s) == 0u);
    /* 大偏角出画 → 不可见 */
    CHECK(camera_project(&fwd, vec3(1.0f, 5.0f, 0.0f), 0.3f, &u, &v, &s) == 0u);

    /* ---- 前视往返：投影 → 重建恢复原相对位置（无噪声） ---- */
    {
        Vec3f rel_nav = vec3(3.0f, 0.8f, -0.4f);
        Quatf att = quat_from_axis_angle(vec3(0.0f, 0.0f, 1.0f), 0.5f);
        Vec3f rel_body = quat_rotate_inv(att, rel_nav);
        CHECK(camera_project(&fwd, rel_body, 0.3f, &u, &v, &s) == 1u);
        Vec3f rec = camera_reconstruct_nav(&fwd, u, v, s, 0.3f, att);
        CHECK(vec3_dist(rec, rel_nav) < 1e-3f);
    }

    /* ---- 下视往返：带姿态倾斜 ---- */
    {
        Vec3f rel_nav = vec3(0.2f, -0.3f, -1.5f);   /* 目标在下方 */
        Quatf att = quat_from_zdir_yaw(vec3(0.1f, 0.05f, 9.8f), 1.0f);
        Vec3f rel_body = quat_rotate_inv(att, rel_nav);
        CHECK(camera_project(&down, rel_body, 0.25f, &u, &v, &s) == 1u);
        Vec3f rec = camera_reconstruct_nav(&down, u, v, s, 0.25f, att);
        CHECK(vec3_dist(rec, rel_nav) < 1e-3f);
    }

    /* 下视相机看不到上方的点 */
    CHECK(camera_project(&down, vec3(0.0f, 0.0f, 1.0f), 0.25f, &u, &v, &s) == 0u);

    /* ---- 尺寸 → 距离关系：尺寸减半 → 距离翻倍 ---- */
    {
        Vec3f rec1 = camera_reconstruct_nav(&fwd, 160.0f, 120.0f, 30.0f, 0.3f,
                                            quat_identity());
        Vec3f rec2 = camera_reconstruct_nav(&fwd, 160.0f, 120.0f, 15.0f, 0.3f,
                                            quat_identity());
        CHECK(fabsf(vec3_norm(rec2) / vec3_norm(rec1) - 2.0f) < 1e-3f);
    }

    if (failures == 0) {
        printf("test_camera: PASS\n");
        return 0;
    }
    printf("test_camera: %d FAILURES\n", failures);
    return 1;
}
