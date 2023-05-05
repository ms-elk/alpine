#pragma once

#include "vector.h"

#include <stdint.h>
#include <vector>

namespace alpine {
class Texture
{
public:
    Texture(uint32_t width, uint32_t height, const uint8_t* data)
        : mWidth(width), mHeight(height)
    {
        mData.resize(width * height);
        for (uint32_t i = 0; i < mData.size(); ++i)
        {
            const uint8_t* d = &data[4 * i];
            mData[i] = float4(d[0], d[1], d[2], d[3]) / 255.0f;
        }
    }

    // TODO
    float4 sample(float2 uv)
    {
        float u = uv.x - static_cast<int32_t>(uv.x);
        float v = uv.y - static_cast<int32_t>(uv.y);
        u = u < 0.0f ? 1.0f + u : u;
        v = v < 0.0f ? - v : 1.0f - v;
        uint32_t x = static_cast<uint32_t>(u * (mWidth - 1));
        uint32_t y = static_cast<uint32_t>(v * (mHeight - 1));
        uint32_t index = x + y * mWidth;
        //return float4(u, v, 0.0f, 0.0f);
        return mData[index];
    }

private:
    uint32_t mWidth;
    uint32_t mHeight;
    std::vector<float4> mData;
};
}