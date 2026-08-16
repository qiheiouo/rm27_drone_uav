#include "camera.h"

/* 前视：cam_x = -body_y, cam_y = -body_z, cam_z = body_x（相机沿机体 +x 看） */
const float CAM_MOUNT_FORWARD[9] = {
    0.0f, -1.0f,  0.0f,
    0.0f,  0.0f, -1.0f,
    1.0f,  0.0f,  0.0f
};

/* 下视：cam_x = body_y, cam_y = body_x, cam_z = -body_z（相机沿机体 -z 看） */
const float CAM_MOUNT_DOWN[9] = {
    0.0f, 1.0f,  0.0f,
    1.0f, 0.0f,  0.0f,
    0.0f, 0.0f, -1.0f
};

void camera_init(CameraModel *cam, float fx, float fy, float cx, float cy,
                 float width, float height, const float mount[9])
{
    int i;
    cam->fx = fx; cam->fy = fy;
    cam->cx = cx; cam->cy = cy;
    cam->width = width; cam->height = height;
    for (i = 0; i < 9; i++) {
        cam->m[i] = mount[i];
    }
}

uint8_t camera_project(const CameraModel *cam, Vec3f rel_body, float obj_size,
                       float *u, float *v, float *size_px)
{
    const float *m = cam->m;
    float px = m[0] * rel_body.x + m[1] * rel_body.y + m[2] * rel_body.z;
    float py = m[3] * rel_body.x + m[4] * rel_body.y + m[5] * rel_body.z;
    float pz = m[6] * rel_body.x + m[7] * rel_body.y + m[8] * rel_body.z;

    if (pz <= 0.05f) {
        return 0u;   /* 在相机后方或过近 */
    }
    *u = cam->cx + cam->fx * px / pz;
    *v = cam->cy + cam->fy * py / pz;
    *size_px = cam->fx * obj_size / pz;

    if (*u < 0.0f || *u >= cam->width || *v < 0.0f || *v >= cam->height) {
        return 0u;   /* 出画 */
    }
    return 1u;
}

Vec3f camera_reconstruct_nav(const CameraModel *cam, float u, float v,
                             float size_px, float obj_size, Quatf att)
{
    float depth;
    Vec3f rel_cam, rel_body;

    if (size_px < 1e-3f) {
        return vec3_zero();
    }
    depth = cam->fx * obj_size / size_px;
    rel_cam = vec3((u - cam->cx) / cam->fx * depth,
                   (v - cam->cy) / cam->fy * depth,
                   depth);

    /* rel_body = M^T * rel_cam（M 为旋转矩阵，转置即逆） */
    {
        const float *m = cam->m;
        rel_body = vec3(m[0] * rel_cam.x + m[3] * rel_cam.y + m[6] * rel_cam.z,
                        m[1] * rel_cam.x + m[4] * rel_cam.y + m[7] * rel_cam.z,
                        m[2] * rel_cam.x + m[5] * rel_cam.y + m[8] * rel_cam.z);
    }
    return quat_rotate(att, rel_body);
}
