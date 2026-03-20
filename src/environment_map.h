#pragma once

#include <math/vector.h>

#include <vector>

namespace alpine {
class EnvironmentMap
{
public:
    EnvironmentMap(std::vector<float3>&& data, uint32_t width, uint32_t height);

    float3 sample(const float3& dir) const;

private:
    std::vector<float3> mData;
    uint32_t mWidth;
    uint32_t mHeight;
};
}
