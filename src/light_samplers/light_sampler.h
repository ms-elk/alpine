#pragma once

#include <math/vector.h>

#include <memory>
#include <optional>
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

    virtual std::optional<Sample> sample(float u, const float3& hit, const float3& ns) const = 0;
};
}
