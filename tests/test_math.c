/* test_math.c - nav_math 基础单元测试 */
#include <stdio.h>
#include <math.h>
#include "nav_math.h"

static int failures = 0;

#define CHECK(cond) do { \
    if (!(cond)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

int main(void)
{
    Vec3f a = vec3(1.0f, 2.0f, 3.0f);
    Vec3f b = vec3(4.0f, -5.0f, 6.0f);

    Vec3f s = vec3_add(a, b);
    CHECK(fabsf(s.x - 5.0f) < 1e-6f && fabsf(s.y + 3.0f) < 1e-6f && fabsf(s.z - 9.0f) < 1e-6f);

    Vec3f d = vec3_sub(b, a);
    CHECK(fabsf(d.x - 3.0f) < 1e-6f && fabsf(d.y + 7.0f) < 1e-6f && fabsf(d.z - 3.0f) < 1e-6f);

    CHECK(fabsf(vec3_dot(a, b) - (4.0f - 10.0f + 18.0f)) < 1e-5f);
    CHECK(fabsf(vec3_norm(vec3(3.0f, 4.0f, 0.0f)) - 5.0f) < 1e-5f);
    CHECK(fabsf(vec3_dist(vec3(0, 0, 0), vec3(0, 0, 2.0f)) - 2.0f) < 1e-5f);

    CHECK(clampf(5.0f, 0.0f, 3.0f) == 3.0f);
    CHECK(clampf(-1.0f, 0.0f, 3.0f) == 0.0f);
    CHECK(clampf(2.0f, 0.0f, 3.0f) == 2.0f);

    CHECK(fabsf(wrap_pi(3.0f * NAV_PI) - NAV_PI) < 1e-4f);
    /* -π 映射到 ±π 均可接受 */
    CHECK(fabsf(fabsf(wrap_pi(-3.0f * NAV_PI)) - NAV_PI) < 1e-4f);
    CHECK(fabsf(wrap_pi(0.3f) - 0.3f) < 1e-6f);

    Vec3f c = vec3_clamp_norm(vec3(10.0f, 0.0f, 0.0f), 2.0f);
    CHECK(fabsf(vec3_norm(c) - 2.0f) < 1e-5f);
    Vec3f e = vec3_clamp_norm(vec3(1.0f, 0.0f, 0.0f), 2.0f);
    CHECK(fabsf(e.x - 1.0f) < 1e-6f);

    if (failures == 0) {
        printf("test_math: PASS\n");
        return 0;
    }
    printf("test_math: %d FAILURES\n", failures);
    return 1;
}
