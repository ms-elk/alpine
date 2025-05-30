static const uniform uint CHILD_NODE_COUNT = 4;
static const uniform float CORRECTION = 1.0000004f; // 1.0f + 2.0f * gamma(3)

struct BoundingBox4
{
    uniform float minX[CHILD_NODE_COUNT];
    uniform float minY[CHILD_NODE_COUNT];
    uniform float minZ[CHILD_NODE_COUNT];
    uniform float maxX[CHILD_NODE_COUNT];
    uniform float maxY[CHILD_NODE_COUNT];
    uniform float maxZ[CHILD_NODE_COUNT];
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
intersectBoundingBox4(
    uniform bool intersects[CHILD_NODE_COUNT],
    const uniform float rayOrg[3],
    const uniform float rayDir[3],
    const uniform float invRayDir[3],
    uniform float tFar,
    const uniform BoundingBox4& bbox4)
{
    foreach(i = 0 ... CHILD_NODE_COUNT) {
        intersects[i] = intersectBoundingBox(
            rayOrg, rayDir, invRayDir, tFar,
            bbox4.minX[i], bbox4.minY[i], bbox4.minZ[i],
            bbox4.maxX[i], bbox4.maxY[i], bbox4.maxZ[i]);
    }
}
