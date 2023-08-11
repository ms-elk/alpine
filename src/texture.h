#pragma once

#include "vector.h"

#include <algorithm>
#include <stdint.h>
#include <vector>

namespace alpine {
template <typename T>
class Texture
{
public:
    Texture(uint32_t width, uint32_t height, std::vector<T>&& data)
        : mWidth(width), mHeight(height), mData(std::move(data))
    {
    }

    T sample(float2 uv)
    {
        // Clip to [0, 1]^2
        float u = uv.x - std::floor(uv.x);
        float v = uv.y - std::floor(uv.y);

        // Scale to [0, w-1] x [0, h-1]
        uint32_t x = static_cast<uint32_t>(u * mWidth);
        uint32_t y = static_cast<uint32_t>(v * mHeight);
        x = std::clamp(x, 0u, mWidth - 1);
        y = std::clamp(y, 0u, mHeight - 1);
        //y = mHeight - 1 - y;

        uint32_t index = x + y * mWidth;
        T value = mData[index];

        return value;
    }

private:
    uint32_t mWidth;
    uint32_t mHeight;
    std::vector<T> mData;
};
}