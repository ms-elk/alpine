#include "power_light_sampler.h"

#include "lights/light.h"

namespace alpine {
PowerLightSampler::PowerLightSampler(const std::vector<std::shared_ptr<Light>>& lights)
    : mLights(lights)
{
    std::vector<float> weights(lights.size());

    for (uint32_t i = 0; i < lights.size(); ++i)
    {
        weights[i] = length(lights[i]->getPower());
    }

    mAliasTable = AliasTable(weights);
}

LightSampler::Sample
PowerLightSampler::sample(float u) const
{
    if (mLights.empty())
    {
        return { nullptr, 0.0f };
    }

    auto ats = mAliasTable.sample(u);

    return { mLights[ats.idx].get(), ats.pdf };
}
}
