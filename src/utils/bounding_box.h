#pragma once

#include "math/vector.h"
#include "ray.h"
#include "utils/util.h"

#include <algorithm>

namespace alpine {
struct BoundingBox
{
    float3 min = std::numeric_limits<float>::max();
    float3 max = -std::numeric_limits<float>::max();

    float3 getCenter() const { return (min + max) * 0.5f; }
    float3 getDiagonal() const { return max - min; };
    float computeSurfaceArea() const
    {
        float3 len = max - min;
        return (len.x * len.y + len.y * len.z + len.z * len.x) * 2.0f;
    }

    bool intersect(const Ray& ray, float tFar, const float3& invRayDir) const
    {
        static constexpr float correction = 1.0f + 2.0f * gamma(3); // ensure conservative intersection

        float tMin = ((invRayDir.x >= 0.0f ? min.x : max.x) - ray.org.x) * invRayDir.x;
        float tMax = ((invRayDir.x >= 0.0f ? max.x : min.x) - ray.org.x) * invRayDir.x;
        tMax *= correction;

        float tyMin = ((invRayDir.y >= 0.0f ? min.y : max.y) - ray.org.y) * invRayDir.y;
        float tyMax = ((invRayDir.y >= 0.0f ? max.y : min.y) - ray.org.y) * invRayDir.y;
        tyMax *= correction;

        if (tMin > tyMax || tMax < tyMin)
        {
            return false;
        }

        tMin = std::max(tMin, tyMin);
        tMax = std::min(tMax, tyMax);

        float tzMin = ((invRayDir.z >= 0.0f ? min.z : max.z) - ray.org.z) * invRayDir.z;
        float tzMax = ((invRayDir.z >= 0.0f ? max.z : min.z) - ray.org.z) * invRayDir.z;
        tzMax *= correction;

        if (tMin > tzMax || tMax < tzMin)
        {
            return false;
        }

        tMin = std::max(tMin, tzMin);
        tMax = std::min(tMax, tzMax);

        return tMin < tFar && tMax > 0.0f;
    }
};

inline BoundingBox
merge(const BoundingBox& b0, const BoundingBox& b1)
{
    BoundingBox bbox;
    bbox.min = min(b0.min, b1.min);
    bbox.max = max(b0.max, b1.max);

    return bbox;
}

inline BoundingBox
merge(const BoundingBox& b, const float3& v)
{
    BoundingBox bbox;
    bbox.min = min(b.min, v);
    bbox.max = max(b.max, v);

    return bbox;
}
}
