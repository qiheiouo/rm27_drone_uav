/*
 * test_vision_frontend.c - 光流里程计单元测试
 *
 * 合成地面点 + 精确投影生成两帧，验证：
 *   1. 纯平移恢复速度
 *   2. 纯旋转被正确补偿（速度≈0）
 *   3. 特征不足 → invalid
 *   4. 外点剔除
 */
#include <stdio.h>
#include <math.h>
#include "vision_frontend.h"

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

#define NP 128

static Vec3f g_points[NP];

static void make_points(void)
{
    /* 确定性网格：16x8，间距 0.4 m，z=0（覆盖 6 m 飞行范围） */
    int i, j, k = 0;
    for (i = 0; i < 16; i++) {
        for (j = 0; j < 8; j++) {
            g_points[k++] = vec3((float)i * 0.4f - 1.4f, (float)j * 0.4f - 1.4f, 0.0f);
        }
    }
}

/* 从相机位姿（pos, att）投影生成一帧（无噪声） */
static void make_frame(const CameraModel *cam, Vec3f cam_pos, Quatf att,
                       uint32_t t_ms, FlowFrame *out)
{
    int i;
    out->count = 0u;
    out->timestamp_ms = t_ms;
    for (i = 0; i < NP && out->count < VF_MAX_FEATURES; i++) {
        Vec3f rel_nav = vec3_sub(g_points[i], cam_pos);
        Vec3f rel_body = quat_rotate_inv(att, rel_nav);
        float u, v, s;
        if (camera_project(cam, rel_body, 0.02f, &u, &v, &s)) {
            out->feats[out->count].id = (uint16_t)i;
            out->feats[out->count].u = u;
            out->feats[out->count].v = v;
            out->count++;
        }
    }
}

static void default_cfg(FlowConfig *cfg, CameraModel *cam)
{
    cfg->min_height = 0.05f;
    cfg->max_height = 5.0f;
    cfg->min_features = 6u;
    cfg->outlier_residual_px = 3.0f;
    camera_init(cam, 180.0f, 180.0f, 160.0f, 120.0f, 320.0f, 240.0f, CAM_MOUNT_DOWN);
}

int main(void)
{
    const float dt = 0.01f;
    FlowConfig cfg;
    CameraModel cam;
    default_cfg(&cfg, &cam);
    make_points();

    /* ---- 1. 纯平移：vx=1.0, vy=-0.5, h=1.0 ---- */
    {
        VisionFrontend vf;
        vf_init(&vf, &cfg, &cam);
        Quatf att = quat_identity();
        Vec3f v_true = vec3(1.0f, -0.5f, 0.0f);
        Vec3f p0 = vec3(0.0f, 0.0f, 1.0f);

        FlowFrame f1, f2;
        make_frame(&cam, p0, att, 0u, &f1);
        make_frame(&cam, vec3_add(p0, vec3_scale(v_true, dt)), att, 10u, &f2);

        OdomSample out;
        vf_update(&vf, &f1, vec3_zero(), att, 1.0f, dt, &out);
        CHECK(out.valid == 0u);   /* 首帧无关联 */
        vf_update(&vf, &f2, vec3_zero(), att, 1.0f, dt, &out);
        CHECK(out.valid == 1u);
        CHECK(vec3_dist(out.vel, v_true) < 0.05f);
        printf("test_vision_frontend: translation vel=(%.3f,%.3f,%.3f) PASS\n",
               out.vel.x, out.vel.y, out.vel.z);
    }

    /* ---- 2. 纯偏航旋转：ωz=0.5 rad/s，位置不动 → 速度≈0 ---- */
    {
        VisionFrontend vf;
        vf_init(&vf, &cfg, &cam);
        Vec3f p0 = vec3(0.0f, 0.0f, 1.0f);
        Quatf a1 = quat_identity();
        Quatf a2 = quat_from_axis_angle(vec3(0.0f, 0.0f, 1.0f), 0.5f * dt);
        Vec3f gyro = vec3(0.0f, 0.0f, 0.5f);

        FlowFrame f1, f2;
        make_frame(&cam, p0, a1, 0u, &f1);
        make_frame(&cam, p0, a2, 10u, &f2);

        OdomSample out;
        vf_update(&vf, &f1, gyro, a1, 1.0f, dt, &out);
        vf_update(&vf, &f2, gyro, a2, 1.0f, dt, &out);
        CHECK(out.valid == 1u);
        CHECK(vec3_norm(out.vel) < 0.05f);
        printf("test_vision_frontend: rotation compensated |v|=%.4f PASS\n",
               vec3_norm(out.vel));
    }

    /* ---- 3. 特征不足 → invalid ---- */
    {
        VisionFrontend vf;
        vf_init(&vf, &cfg, &cam);
        Quatf att = quat_identity();
        FlowFrame f1, f2;
        f1.count = 3u; f1.timestamp_ms = 0u;   /* 故意只有 3 个特征 */
        f2.count = 3u; f2.timestamp_ms = 10u;
        for (int i = 0; i < 3; i++) {
            f1.feats[i].id = (uint16_t)i; f1.feats[i].u = 100.0f; f1.feats[i].v = 100.0f;
            f2.feats[i].id = (uint16_t)i; f2.feats[i].u = 101.0f; f2.feats[i].v = 100.0f;
        }
        OdomSample out;
        vf_update(&vf, &f1, vec3_zero(), att, 1.0f, dt, &out);
        vf_update(&vf, &f2, vec3_zero(), att, 1.0f, dt, &out);
        CHECK(out.valid == 0u);
        printf("test_vision_frontend: too-few-features invalid PASS\n");
    }

    /* ---- 4. 外点剔除：一个特征错位 20 px ---- */
    {
        VisionFrontend vf;
        vf_init(&vf, &cfg, &cam);
        Quatf att = quat_identity();
        Vec3f v_true = vec3(0.8f, 0.3f, 0.0f);
        Vec3f p0 = vec3(0.0f, 0.0f, 1.0f);

        FlowFrame f1, f2;
        make_frame(&cam, p0, att, 0u, &f1);
        make_frame(&cam, vec3_add(p0, vec3_scale(v_true, dt)), att, 10u, &f2);
        f2.feats[0].u += 20.0f;   /* 注入外点 */

        OdomSample out;
        vf_update(&vf, &f1, vec3_zero(), att, 1.0f, dt, &out);
        vf_update(&vf, &f2, vec3_zero(), att, 1.0f, dt, &out);
        CHECK(out.valid == 1u);
        CHECK(vec3_dist(out.vel, v_true) < 0.08f);
        printf("test_vision_frontend: outlier rejected PASS\n");
    }

    /* ---- 5. 位置积分：匀速 1 m/s 飞 2 s，VO 位置≈2 m ---- */
    {
        VisionFrontend vf;
        vf_init(&vf, &cfg, &cam);
        Quatf att = quat_identity();
        Vec3f v_true = vec3(1.0f, 0.0f, 0.0f);
        Vec3f pos = vec3(0.0f, 0.0f, 1.0f);

        OdomSample out;
        out.valid = 0u;
        uint32_t t = 0u;
        for (int k = 0; k < 200; k++) {
            FlowFrame f;
            make_frame(&cam, pos, att, t, &f);
            vf_update(&vf, &f, vec3_zero(), att, 1.0f, dt, &out);
            pos = vec3_add(pos, vec3_scale(v_true, dt));
            t += 10u;
        }
        CHECK(out.valid == 1u);
        CHECK(fabsf(out.pos.x - 2.0f) < 0.05f);
        printf("test_vision_frontend: 2s integration x=%.3f PASS\n", out.pos.x);
    }

    if (failures == 0) {
        printf("test_vision_frontend: PASS\n");
        return 0;
    }
    printf("test_vision_frontend: %d FAILURES\n", failures);
    return 1;
}
