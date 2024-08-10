#pragma once

#include "light_sampler.h"

namespace alpine {
class BvhLightSampler : public LightSampler
{
public:
    BvhLightSampler(const std::vector<std::shared_ptr<Light>>& lights);

    Sample sample(float u, const float3& hit) const override;

private:
    struct LightBvh;
    std::unique_ptr<LightBvh> bvh;
};
}
