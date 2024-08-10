#pragma once

#include "light_sampler.h"

namespace alpine {
class UniformLightSampler : public LightSampler
{
public:
    UniformLightSampler(const std::vector<std::shared_ptr<Light>>& lights);

    Sample sample(float u, const float3& hit, const float3& ns) const override;

private:
    std::vector<Light*> mLights;
    float mPdf = 0.0f;
};
}
