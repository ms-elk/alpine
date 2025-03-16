#pragma once

#include "light_sampler.h"

namespace alpine {
class BvhLightSampler final : public LightSampler
{
public:
    BvhLightSampler(const std::vector<std::shared_ptr<Light>>& lights);
    ~BvhLightSampler() override;

    std::optional<Sample> sample(float u, const float3& hit, const float3& ns) const override;

private:
    struct LightBvh;
    std::unique_ptr<LightBvh> bvh;
};
}
