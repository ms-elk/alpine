#pragma once

#include "math/vector.h"
#include "ray.h"

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

    bool intersect(const Ray& ray) const
    {
        float tNear = -std::numeric_limits<float>::max();
        float tFar = std::numeric_limits<float>::max();

        for (uint32_t i = 0; i < 3; ++i)
        {
            if (std::abs(ray.dir[i]) > 0.0f)
            {
                float t0 = (min[i] - ray.org[i]) / ray.dir[i];
                float t1 = (max[i] - ray.org[i]) / ray.dir[i];

                if (t0 > t1)
                {
                    std::swap(t0, t1);
                }

                tNear = std::max(tNear, t0);
                tFar = std::min(tFar, t1);

                if (tNear > tFar)
                {
                    return false;
                }
            }
            else
            {
                if (ray.org[i] < min[i] || ray.org[i] > max[i])
                {
                    return false;
                }
            }
        }

        return true;
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
