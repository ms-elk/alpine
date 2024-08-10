#pragma once

#include "math/vector.h"

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
};

inline BoundingBox
merge(const BoundingBox b0, const BoundingBox b1)
{
    BoundingBox bbox;
    bbox.min = min(b0.min, b1.min);
    bbox.max = max(b0.max, b1.max);

    return bbox;
}
}
