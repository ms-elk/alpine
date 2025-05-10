#include "power_light_sampler.h"

#include <lights/light.h>

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

std::optional<LightSampler::Sample>
PowerLightSampler::sample(float u, const float3& hit, const float3& ns) const
{
    if (mLights.empty())
    {
        return {};
    }

    auto ats = mAliasTable.sample(u);

    return Sample{ mLights[ats.idx], ats.pdf };
}
}
