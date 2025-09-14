static const uniform uint TRIANGLE_COUNT = 4;
static const uniform float NO_INTERSECT = FLT_MAX;
static const uniform float EPSILON = 1e-5f;

struct Intersection4
{
    uniform float t[TRIANGLE_COUNT];
    
    uniform float b0[TRIANGLE_COUNT];
    uniform float b1[TRIANGLE_COUNT];
};

struct Triangle4
{
    uniform float vx[TRIANGLE_COUNT];
    uniform float vy[TRIANGLE_COUNT];
    uniform float vz[TRIANGLE_COUNT];
    
    uniform float ex0[TRIANGLE_COUNT];
    uniform float ey0[TRIANGLE_COUNT];
    uniform float ez0[TRIANGLE_COUNT];
    uniform float ex1[TRIANGLE_COUNT];
    uniform float ey1[TRIANGLE_COUNT];
    uniform float ez1[TRIANGLE_COUNT];
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
intersectTriangle4(
    uniform Intersection4& isects,
    const uniform float rayOrg[3],
    const uniform float rayDir[3],
    const uniform Triangle4& tri4)
{
    foreach(i = 0... TRIANGLE_COUNT)
    {
        float sx = rayOrg[0] - tri4.vx[i];
        float sy = rayOrg[1] - tri4.vy[i];
        float sz = rayOrg[2] - tri4.vz[i];
    
        float sx1, sy1, sz1;
        cross(
            rayDir[0], rayDir[1], rayDir[2],
            tri4.ex1[i], tri4.ey1[i], tri4.ez1[i],
            sx1, sy1, sz1);

        float sx2, sy2, sz2;
        cross(
            sx, sy, sz,
            tri4.ex0[i], tri4.ey0[i], tri4.ez0[i],
            sx2, sy2, sz2);

        float det = dot(sx1, sy1, sz1, tri4.ex0[i], tri4.ey0[i], tri4.ez0[i]);
        
        bool isIntersected = abs(det) >= EPSILON;

        float rcpDet = 1.0f / det;
        float t = dot(sx2, sy2, sz2, tri4.ex1[i], tri4.ey1[i], tri4.ez1[i]) * rcpDet;
        float b0 = dot(sx1, sy1, sz1, sx, sy, sz) * rcpDet;
        float b1 = dot(rayDir[0], rayDir[1], rayDir[2], sx2, sy2, sz2) * rcpDet;

        isIntersected &= t >= 0.0f;
        isIntersected &= (b0 >= 0.0f) & (b1 >= 0.0f) & (b0 + b1 <= 1.0f);

        isects.t[i] = select(isIntersected, t, NO_INTERSECT);
        isects.b0[i] = select(isIntersected, b0, NO_INTERSECT);
        isects.b1[i] = select(isIntersected, b1, NO_INTERSECT);
    }
}
