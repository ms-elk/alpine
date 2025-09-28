#include "ispc_config.h"

static const uniform float EPSILON = 1e-5f;

struct IntersectionN
{
    uniform float t[SIMD_WIDTH];
    
    uniform float b0[SIMD_WIDTH];
    uniform float b1[SIMD_WIDTH];
};

struct TriangleN
{
    uniform float vx[SIMD_WIDTH];
    uniform float vy[SIMD_WIDTH];
    uniform float vz[SIMD_WIDTH];
    
    uniform float ex0[SIMD_WIDTH];
    uniform float ey0[SIMD_WIDTH];
    uniform float ez0[SIMD_WIDTH];
    uniform float ex1[SIMD_WIDTH];
    uniform float ey1[SIMD_WIDTH];
    uniform float ez1[SIMD_WIDTH];
};

static inline void
cross(
    float vx0, float vy0, float vz0,
    float vx1, float vy1, float vz1,
    float& vx, float& vy, float& vz)
{
    vx = vy0 * vz1 - vz0 * vy1;
    vy = vz0 * vx1 - vx0 * vz1;
    vz = vx0 * vy1 - vy0 * vx1;
}

static inline float
dot(
    float vx0, float vy0, float vz0,
    float vx1, float vy1, float vz1)
{
    return vx0 * vx1 + vy0 * vy1 + vz0 * vz1;
}

export void
intersectTriangleN(
    uniform IntersectionN& isects,
    const uniform float rayOrg[3],
    const uniform float rayDir[3],
    const uniform TriangleN& triN)
{
    foreach(i = 0 ... SIMD_WIDTH)
    {
        float sx = rayOrg[0] - triN.vx[i];
        float sy = rayOrg[1] - triN.vy[i];
        float sz = rayOrg[2] - triN.vz[i];
    
        float sx1, sy1, sz1;
        cross(
            rayDir[0], rayDir[1], rayDir[2],
            triN.ex1[i], triN.ey1[i], triN.ez1[i],
            sx1, sy1, sz1);

        float sx2, sy2, sz2;
        cross(
            sx, sy, sz,
            triN.ex0[i], triN.ey0[i], triN.ez0[i],
            sx2, sy2, sz2);

        float det = dot(sx1, sy1, sz1, triN.ex0[i], triN.ey0[i], triN.ez0[i]);
        
        bool isIntersected = abs(det) >= EPSILON;

        float rcpDet = 1.0f / det;
        float t = dot(sx2, sy2, sz2, triN.ex1[i], triN.ey1[i], triN.ez1[i]) * rcpDet;
        float b0 = dot(sx1, sy1, sz1, sx, sy, sz) * rcpDet;
        float b1 = dot(rayDir[0], rayDir[1], rayDir[2], sx2, sy2, sz2) * rcpDet;

        isIntersected &= t >= 0.0f;
        isIntersected &= (b0 >= 0.0f) & (b1 >= 0.0f) & (b0 + b1 <= 1.0f);

        isects.t[i] = select(isIntersected, t, NO_INTERSECT);
        isects.b0[i] = select(isIntersected, b0, NO_INTERSECT);
        isects.b1[i] = select(isIntersected, b1, NO_INTERSECT);
    }
}
