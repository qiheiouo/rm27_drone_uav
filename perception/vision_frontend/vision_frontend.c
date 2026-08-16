#include "vision_frontend.h"

void vf_init(VisionFrontend *vf, const FlowConfig *cfg, const CameraModel *cam)
{
    vf->cfg = *cfg;
    vf->cam = *cam;
    vf->prev_count = 0u;
    vf->has_prev = 0u;
    vf->vo_pos = vec3_zero();
    vf->vo_yaw = 0.0f;
    vf->last_matched = 0u;
    vf->last_mean_residual = 0.0f;
}

void vf_reset_track(VisionFrontend *vf)
{
    vf->prev_count = 0u;
    vf->has_prev = 0u;
}

void vf_set_pose(VisionFrontend *vf, Vec3f pos, float yaw)
{
    vf->vo_pos = pos;
    vf->vo_yaw = yaw;
}

/* 3x3 正规方程闭式求解：theta = (A^T A)^-1 A^T b */
static uint8_t solve3(const float ata[9], const float atb[3], float theta[3])
{
    float a = ata[0], b = ata[1], c = ata[2];
    float d = ata[4], e = ata[5];
    float f = ata[8];
    float A =  d*f - e*e;
    float B = -(b*f - c*e);
    float C =  b*e - c*d;
    float det = a*A + b*B + c*C;
    if (fabsf(det) < 1e-9f) {
        return 0u;
    }
    float inv = 1.0f / det;
    /* A^T A 对称，余子式即伴随 */
    float m00 = A,        m01 = B,        m02 = C;
    float m10 = B;
    float m11 = a*f - c*c;
    float m12 = -(a*e - b*c);
    float m22 = a*d - b*b;
    float m02_ = m02, m12_ = m12;
    theta[0] = (m00 * atb[0] + m01 * atb[1] + m02_ * atb[2]) * inv;
    theta[1] = (m10 * atb[0] + m11 * atb[1] + m12_ * atb[2]) * inv;
    theta[2] = (m02_ * atb[0] + m12_ * atb[1] + m22 * atb[2]) * inv;
    return 1u;
}

/*
 * 一次最小二乘：由去旋转后的光流解相机系速度 (Vx,Vy,Vz)。
 * matched 数组：n 对 (x,y,du,dv)，h 为离地高度。
 */
static uint8_t flow_ls(const VisionFrontend *vf,
                       const float *xs, const float *ys,
                       const float *dus, const float *dvs,
                       uint8_t n, float h, float vel_cam[3],
                       float *mean_residual)
{
    float ata[9] = {0.0f};
    float atb[3] = {0.0f};
    float fx = vf->cam.fx, fy = vf->cam.fy;
    uint8_t i;

    if (n < vf->cfg.min_features) {
        return 0u;
    }

    for (i = 0u; i < n; i++) {
        /* du = fx(-Vx + x Vz)/h → 行向量 a_u = (-fx/h, 0, fx*x/h)
         * dv = fy(-Vy + y Vz)/h → a_v = (0, -fy/h, fy*y/h) */
        float au[3] = { -fx / h, 0.0f, fx * xs[i] / h };
        float av[3] = { 0.0f, -fy / h, fy * ys[i] / h };
        int r, c;
        for (r = 0; r < 3; r++) {
            for (c = 0; c < 3; c++) {
                ata[r * 3 + c] += au[r] * au[c] + av[r] * av[c];
            }
            atb[r] += au[r] * dus[i] + av[r] * dvs[i];
        }
    }

    if (!solve3(ata, atb, vel_cam)) {
        return 0u;
    }

    /* 平均残差 */
    {
        float res_sum = 0.0f;
        for (i = 0u; i < n; i++) {
            float pu = (-fx * vel_cam[0] + fx * xs[i] * vel_cam[2]) / h;
            float pv = (-fy * vel_cam[1] + fy * ys[i] * vel_cam[2]) / h;
            float ru = dus[i] - pu;
            float rv = dvs[i] - pv;
            res_sum += sqrtf(ru * ru + rv * rv);
        }
        *mean_residual = res_sum / (float)n;
    }
    return 1u;
}

void vf_update(VisionFrontend *vf, const FlowFrame *frame,
               Vec3f gyro_body, Quatf att, float height, float dt,
               OdomSample *out)
{
    float fx = vf->cam.fx, fy = vf->cam.fy;
    float cx = vf->cam.cx, cy = vf->cam.cy;
    const float *m = vf->cam.m;

    float xs[VF_MAX_FEATURES], ys[VF_MAX_FEATURES];
    float dus[VF_MAX_FEATURES], dvs[VF_MAX_FEATURES];
    uint16_t m_id[VF_MAX_FEATURES];
    float    m_u[VF_MAX_FEATURES], m_v[VF_MAX_FEATURES];
    uint8_t n = 0u;
    uint8_t i, j;

    out->valid = 0u;
    out->timestamp_ms = frame->timestamp_ms;
    out->pos = vf->vo_pos;
    out->vel = vec3_zero();
    out->yaw = vf->vo_yaw;
    out->yaw_rate = 0.0f;
    out->att = att;

    /* ---- 特征关联（id 匹配，小容量线性扫描） ---- */
    if (vf->has_prev && dt > 1e-4f) {
        for (i = 0u; i < frame->count && i < VF_MAX_FEATURES; i++) {
            for (j = 0u; j < vf->prev_count; j++) {
                if (vf->prev_id[j] == frame->feats[i].id && n < VF_MAX_FEATURES) {
                    m_id[n] = frame->feats[i].id;
                    m_u[n]  = frame->feats[i].u;
                    m_v[n]  = frame->feats[i].v;
                    dus[n]  = frame->feats[i].u - vf->prev_u[j];
                    dvs[n]  = frame->feats[i].v - vf->prev_v[j];
                    n++;
                    break;
                }
            }
        }
    }

    /* ---- 光流解算 ---- */
    uint8_t ok = 0u;
    float vel_cam[3] = {0.0f, 0.0f, 0.0f};
    float mean_res = 0.0f;

    if (n >= vf->cfg.min_features &&
        height >= vf->cfg.min_height && height <= vf->cfg.max_height) {

        /* 陀螺 → 相机系 */
        float wx = m[0] * gyro_body.x + m[1] * gyro_body.y + m[2] * gyro_body.z;
        float wy = m[3] * gyro_body.x + m[4] * gyro_body.y + m[5] * gyro_body.z;
        float wz = m[6] * gyro_body.x + m[7] * gyro_body.y + m[8] * gyro_body.z;

        for (i = 0u; i < n; i++) {
            float x = (m_u[i] - cx) / fx;
            float y = (m_v[i] - cy) / fy;
            /* 旋转光流（标准方程），从实测光流中扣除 */
            float du_rot = fx * (x * y * wx - (1.0f + x * x) * wy + y * wz) * dt;
            float dv_rot = fy * ((1.0f + y * y) * wx - x * y * wy - x * wz) * dt;
            xs[i] = x;
            ys[i] = y;
            dus[i] = (dus[i] - du_rot) / dt;   /* 位移 → 速率 */
            dvs[i] = (dvs[i] - dv_rot) / dt;
        }

        ok = flow_ls(vf, xs, ys, dus, dvs, n, height, vel_cam, &mean_res);

        /* 外点剔除后重解一次（残差为速率 px/s，门限换算为 px/帧） */
        if (ok && mean_res > 0.0f) {
            float xs2[VF_MAX_FEATURES], ys2[VF_MAX_FEATURES];
            float dus2[VF_MAX_FEATURES], dvs2[VF_MAX_FEATURES];
            uint8_t n2 = 0u;
            float res_thr_rate = vf->cfg.outlier_residual_px / dt;
            for (i = 0u; i < n; i++) {
                float pu = (-fx * vel_cam[0] + fx * xs[i] * vel_cam[2]) / height;
                float pv = (-fy * vel_cam[1] + fy * ys[i] * vel_cam[2]) / height;
                float ru = dus[i] - pu;
                float rv = dvs[i] - pv;
                if (sqrtf(ru * ru + rv * rv) <= res_thr_rate) {
                    xs2[n2] = xs[i]; ys2[n2] = ys[i];
                    dus2[n2] = dus[i]; dvs2[n2] = dvs[i];
                    n2++;
                }
            }
            if (n2 >= vf->cfg.min_features) {
                float res2 = 0.0f;
                if (flow_ls(vf, xs2, ys2, dus2, dvs2, n2, height, vel_cam, &res2)) {
                    n = n2;
                    mean_res = res2;
                }
            }
        }
    }

    /* ---- 保存本帧供下次关联 ---- */
    vf->prev_count = 0u;
    for (i = 0u; i < frame->count && i < VF_MAX_FEATURES; i++) {
        vf->prev_id[i] = frame->feats[i].id;
        vf->prev_u[i]  = frame->feats[i].u;
        vf->prev_v[i]  = frame->feats[i].v;
        vf->prev_count++;
    }
    vf->has_prev = (frame->count > 0u) ? 1u : 0u;
    vf->last_matched = n;
    vf->last_mean_residual = mean_res;

    if (!ok) {
        return;
    }

    /* ---- 相机系速度 → 机体系 → 导航系，积分位置 ---- */
    {
        Vec3f v_cam = vec3(vel_cam[0], vel_cam[1], vel_cam[2]);
        Vec3f v_body = vec3(m[0] * v_cam.x + m[3] * v_cam.y + m[6] * v_cam.z,
                            m[1] * v_cam.x + m[4] * v_cam.y + m[7] * v_cam.z,
                            m[2] * v_cam.x + m[5] * v_cam.y + m[8] * v_cam.z);
        Vec3f v_nav = quat_rotate(att, v_body);

        vf->vo_pos = vec3_add(vf->vo_pos, vec3_scale(v_nav, dt));
        /* 偏航：机体系 z 陀螺积分（近水平飞行时 ≈ 导航系偏航率）。
         * 不用 est 姿态旋转——避免 vf 偏航与 est 偏航形成正反馈回路 */
        vf->vo_yaw = wrap_pi(vf->vo_yaw + gyro_body.z * dt);

        out->valid = 1u;
        out->pos = vf->vo_pos;
        out->vel = v_nav;
        out->yaw = vf->vo_yaw;
        out->yaw_rate = gyro_body.z;
    }
}
