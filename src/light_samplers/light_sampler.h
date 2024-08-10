#pragma once

#include "math/vector.h"

#include <memory>
#include <vector>

namespace alpine {
class Light;

class LightSampler
{
public:
    LightSampler() = default;
    virtual ~LightSampler() = default;

    struct Sample
    {
        Light* light = nullptr;
        float pdf;
    };

    virtual Sample sample(float u, const float3& hit) const = 0;
};
}
