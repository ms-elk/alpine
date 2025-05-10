#pragma once

#include <math/vector.h>

#include <algorithm>

namespace alpine {
struct Intersection
{
    const void* shapePtr = nullptr;
    uint32_t primId = std::numeric_limits<uint32_t>::max();
    float3 ng;
    float2 barycentric;
    float t = std::numeric_limits<float>::max();

};
}
