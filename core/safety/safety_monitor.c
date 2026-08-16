#include "safety_monitor.h"

void safety_init(SafetyMonitor *mon, const SafetyConfig *cfg)
{
    mon->cfg = *cfg;
    mon->elapsed_s = 0.0f;
    mon->time_exceeded = 0u;
    mon->geofence_violation = 0u;
}

void safety_update(SafetyMonitor *mon, const NavState *nav, const Vec3f *home, float dt)
{
    mon->elapsed_s += dt;
    if (mon->elapsed_s >= mon->cfg.max_mission_time_s) {
        mon->time_exceeded = 1u;
    }
    if (vec3_dist_xy(nav->pos, *home) > mon->cfg.geofence_radius_m ||
        nav->pos.z > mon->cfg.geofence_max_alt_m) {
        mon->geofence_violation = 1u;
    }
}
