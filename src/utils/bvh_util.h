#pragma once

#include <math/vector.h>

namespace alpine::bvh_util {
inline uint8_t getBinIndex(float center, float min, float max, uint8_t binCount)
{
    uint8_t binIdx = static_cast<uint8_t>(binCount * (center - min) / (max - min));
    binIdx = std::min(binIdx, static_cast<uint8_t>(binCount - 1));

    return binIdx;
}

struct Split
{
    uint8_t dim = 0;
    uint8_t splitIdx = 0;
    uint8_t binCount = 0;
    float min = 0.0f;
    float max = 0.0f;

    inline bool isBelow(const float3& center) const
    {
        uint8_t binIdx = getBinIndex(center[dim], min, max, binCount);
        return binIdx <= splitIdx;
    }
};
}
