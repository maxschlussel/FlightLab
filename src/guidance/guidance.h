#pragma once


typedef enum {
    GUIDANCE_NONE,              // No guidance (manual or open-loop)
    GUIDANCE_WAYPOINT,          // Waypoint guidance (lat, long, alt)
    GUIDANCE_LOITER,            // Loiter around point
    GUIDANCE_PATH_FOLLOW,       // Follow continuous path
    GUIDANCE_HEADING_ALT_VEL,   // Cruise at a specified heading, altitude, and velocity
    GUIDANCE_GLIDEPATH,         // Follow glidepath between two points
    GUIDANCE_ATTITUDE          // Command specific aircraft attitude (roll, pitch, yaw)
} GuidanceMode;


typedef struct {
    GuidanceMode mode;

    union {
        struct { double lat_deg, lon_deg, alt_m; } latlon;
        struct { double lat_deg, lon_deg, alt_m, radius; int cw; } loiter;
        struct { double x0, y0, z0; double dx, dy, dz; } path;
        struct { double heading, alt, vel; } headingAltVel;
        struct { double x0, y0, z0; double x1, y1, z1; } glidepath;
        struct { double roll, pitch, yaw; } attitude;
    } refs;

} GuidanceRefs;

void handleGuidance(const GuidanceRefs* g);

GuidanceRefs initGuidanceNone(void);

void updateGuidanceRefs(GuidanceRefs* guidanceRefs);

