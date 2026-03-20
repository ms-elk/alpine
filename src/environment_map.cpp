#include "environment_map.h"

#include <numbers>

namespace alpine {
EnvironmentMap::EnvironmentMap(std::vector<float3>&& data, uint32_t width, uint32_t height)
    : mData(std::move(data))
    , mWidth(width)
    , mHeight(height)
{
}

namespace {
float2
toUv(const float3& dir)
{
    float u = std::atan2(dir.z, dir.x) / (2.0f * std::numbers::pi_v<float>) + 0.5f;
    float v = std::acos(std::clamp(dir.y, -1.0f, 1.0f)) / std::numbers::pi_v<float>;

    return { u, v };
}
}

float3
EnvironmentMap::sample(const float3& dir) const
{
    float2 uv = toUv(dir);

    uv.x = uv.x - std::floor(uv.x);
    uv.y = std::clamp(uv.y, 0.0f, 1.0f);

    uint32_t x = std::min(static_cast<uint32_t>(uv.x * mWidth), mWidth - 1);
    uint32_t y = std::min(static_cast<uint32_t>(uv.y * mHeight), mHeight - 1);

    uint32_t index = y * mWidth + x;
    return mData[index];
}
}
