/*
 * nav_math.h - 最小数学库（MCU 兼容）
 *
 * 仅提供固定大小的 Vec3f 与标量工具函数，header-only，
 * 不引入动态内存与外部数学库依赖。
 *
 * 坐标系约定（仿真与算法统一）：
 *   x/y 为水平面，z 垂直向上（ altitude 为正）。
 */
#ifndef NAV_MATH_H
#define NAV_MATH_H

#include <math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NAV_PI 3.14159265f
#define NAV_2PI 6.2831853f
#define NAV_GRAVITY 9.81f

typedef struct {
    float x, y, z;
} Vec3f;

static inline Vec3f vec3(float x, float y, float z)
{
    Vec3f v; v.x = x; v.y = y; v.z = z; return v;
}

static inline Vec3f vec3_zero(void) { return vec3(0.0f, 0.0f, 0.0f); }

static inline Vec3f vec3_add(Vec3f a, Vec3f b) { return vec3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline Vec3f vec3_sub(Vec3f a, Vec3f b) { return vec3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline Vec3f vec3_scale(Vec3f a, float s) { return vec3(a.x * s, a.y * s, a.z * s); }

static inline float vec3_dot(Vec3f a, Vec3f b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline float vec3_norm(Vec3f a) { return sqrtf(vec3_dot(a, a)); }
static inline float vec3_norm_xy(Vec3f a) { return sqrtf(a.x * a.x + a.y * a.y); }
static inline float vec3_dist(Vec3f a, Vec3f b) { return vec3_norm(vec3_sub(a, b)); }
static inline float vec3_dist_xy(Vec3f a, Vec3f b) { return vec3_norm_xy(vec3_sub(a, b)); }

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline float wrap_pi(float a)
{
    while (a > NAV_PI)  a -= NAV_2PI;
    while (a < -NAV_PI) a += NAV_2PI;
    return a;
}

/* 将向量模长限制到 max_norm（方向不变） */
static inline Vec3f vec3_clamp_norm(Vec3f v, float max_norm)
{
    float n = vec3_norm(v);
    if (n > max_norm && n > 1e-6f) {
        return vec3_scale(v, max_norm / n);
    }
    return v;
}

/* ---------------- 四元数（机体 → 导航系姿态） ---------------- */

typedef struct {
    float w, x, y, z;
} Quatf;

static inline Quatf quat_identity(void) { Quatf q = {1.0f, 0.0f, 0.0f, 0.0f}; return q; }

static inline Quatf quat_mul(Quatf a, Quatf b)
{
    Quatf q;
    q.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    q.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    q.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    q.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return q;
}

static inline Quatf quat_conj(Quatf q) { Quatf r = {q.w, -q.x, -q.y, -q.z}; return r; }

static inline Quatf quat_normalize(Quatf q)
{
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n < 1e-9f) { return quat_identity(); }
    float inv = 1.0f / n;
    Quatf r = {q.w*inv, q.x*inv, q.y*inv, q.z*inv};
    return r;
}

/* 轴角 → 四元数（axis 无需归一化；angle 为绕轴转角 rad） */
static inline Quatf quat_from_axis_angle(Vec3f axis, float angle)
{
    float n = vec3_norm(axis);
    if (n < 1e-9f) { return quat_identity(); }
    float h = 0.5f * angle;
    float s = sinf(h) / n;
    Quatf q = {cosf(h), axis.x * s, axis.y * s, axis.z * s};
    return q;
}

/* 旋转向量（陀螺积分）：q_new = q ⊗ delta(omega*dt)，omega 为机体系角速度 */
static inline Quatf quat_integrate_gyro(Quatf q, Vec3f gyro_body, float dt)
{
    float wnorm = vec3_norm(gyro_body);
    if (wnorm < 1e-9f) { return q; }
    Quatf dq = quat_from_axis_angle(gyro_body, wnorm * dt);
    return quat_normalize(quat_mul(q, dq));
}

/* v_nav = q ⊗ v_body ⊗ q* */
static inline Vec3f quat_rotate(Quatf q, Vec3f v)
{
    Vec3f u = vec3(q.x, q.y, q.z);
    Vec3f t = vec3_scale(vec3(u.y*v.z - u.z*v.y,
                              u.z*v.x - u.x*v.z,
                              u.x*v.y - u.y*v.x), 2.0f);
    Vec3f r = vec3_add(v, vec3_add(vec3_scale(t, q.w),
                       vec3(u.y*t.z - u.z*t.y,
                            u.z*t.x - u.x*t.z,
                            u.x*t.y - u.y*t.x)));
    return r;
}

/* v_body = q* ⊗ v_nav ⊗ q */
static inline Vec3f quat_rotate_inv(Quatf q, Vec3f v)
{
    return quat_rotate(quat_conj(q), v);
}

static inline float quat_to_yaw(Quatf q)
{
    return atan2f(2.0f * (q.w*q.z + q.x*q.y),
                  1.0f - 2.0f * (q.y*q.y + q.z*q.z));
}

/*
 * 由推力方向（比力方向，机体 z 轴指向）与偏航构造姿态：
 * q = tilt(z→z_dir) ⊗ yaw，用于仿真的姿态真值模型。
 */
static inline Quatf quat_from_zdir_yaw(Vec3f z_dir, float yaw)
{
    Vec3f d = vec3_clamp_norm(z_dir, 1.0f);
    float dn = vec3_norm(z_dir);
    if (dn > 1e-6f) { d = vec3_scale(z_dir, 1.0f / dn); }
    Quatf q_yaw = quat_from_axis_angle(vec3(0.0f, 0.0f, 1.0f), yaw);
    Quatf q_tilt;
    float dot = d.z;  /* (0,0,1)·d */
    if (dot > 0.99999f) {
        q_tilt = quat_identity();
    } else if (dot < -0.99999f) {
        q_tilt = quat_from_axis_angle(vec3(1.0f, 0.0f, 0.0f), NAV_PI);
    } else {
        Vec3f axis = vec3(-d.y, d.x, 0.0f);
        q_tilt = quat_from_axis_angle(axis, acosf(clampf(dot, -1.0f, 1.0f)));
    }
    return quat_normalize(quat_mul(q_tilt, q_yaw));
}

#ifdef __cplusplus
}
#endif

#endif /* NAV_MATH_H */
