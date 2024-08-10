#include "power_light_sampler.h"

#include "lights/light.h"

namespace alpine {
PowerLightSampler::PowerLightSampler(const std::vector<std::shared_ptr<Light>>& lights)
{
    mLights.resize(lights.size());
    for (uint32_t i = 0; i < lights.size(); ++i)
    {
        mLights[i] = lights[i].get();
    }

    std::vector<float> weights(mLights.size());

    for (uint32_t i = 0; i < mLights.size(); ++i)
    {
        weights[i] = length(mLights[i]->getPower());
    }

    mAliasTable = AliasTable(weights);
}

LightSampler::Sample
PowerLightSampler::sample(float u, const float3& hit) const
{
    if (mLights.empty())
    {
        return { nullptr, 0.0f };
    }

    auto ats = mAliasTable.sample(u);

    return { mLights[ats.idx], ats.pdf };
}
}
