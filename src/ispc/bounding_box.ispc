#include "ispc_config.h"

static const uniform float CORRECTION = 1.0000004f; // 1.0f + 2.0f * gamma(3)

struct BoundingBoxN
{
    uniform float minX[SIMD_WIDTH];
    uniform float minY[SIMD_WIDTH];
    uniform float minZ[SIMD_WIDTH];
    uniform float maxX[SIMD_WIDTH];
    uniform float maxY[SIMD_WIDTH];
    uniform float maxZ[SIMD_WIDTH];
};

static bool
intersectBoundingBox(
    const uniform float rayOrg[3],
    const uniform float rayDir[3],
    const uniform float invRayDir[3],
    uniform float tFar,
    float minX,
    float minY,
    float minZ,
    float maxX,
    float maxY,
    float maxZ)
{
    float tx0 = (minX - rayOrg[0]) * invRayDir[0];
    float tx1 = (maxX - rayOrg[0]) * invRayDir[0];

    float ty0 = (minY - rayOrg[1]) * invRayDir[1];
    float ty1 = (maxY - rayOrg[1]) * invRayDir[1];

    float tz0 = (minZ - rayOrg[2]) * invRayDir[2];
    float tz1 = (maxZ - rayOrg[2]) * invRayDir[2];

    float tMin = max(max(min(tx0, tx1), min(ty0, ty1)), min(tz0, tz1));
    float tMax = min(min(max(tx0, tx1), max(ty0, ty1)), max(tz0, tz1));

    tMax *= CORRECTION; // ensure conservative intersection
    
    return (tMin <= tMax) & (tMax > 0.0f) & (tMin < tFar);
}

export void
intersectBoundingBoxN(
    uniform bool intersects[SIMD_WIDTH],
    const uniform float rayOrg[3],
    const uniform float rayDir[3],
    const uniform float invRayDir[3],
    uniform float tFar,
    const uniform BoundingBoxN& bboxN)
{
    foreach(i = 0 ... SIMD_WIDTH)
    {
        intersects[i] = intersectBoundingBox(
            rayOrg, rayDir, invRayDir, tFar,
            bboxN.minX[i], bboxN.minY[i], bboxN.minZ[i],
            bboxN.maxX[i], bboxN.maxY[i], bboxN.maxZ[i]);
    }
}
