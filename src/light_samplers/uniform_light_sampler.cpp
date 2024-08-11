#include "uniform_light_sampler.h"

namespace alpine {
UniformLightSampler::UniformLightSampler(const std::vector<std::shared_ptr<Light>>& lights)
{
    mLights.resize(lights.size());
    for (uint32_t i = 0; i < lights.size(); ++i)
    {
        mLights[i] = lights[i].get();
    }

    mPdf = !mLights.empty() ? 1.0f / mLights.size() : 0.0f;
}

std::optional<LightSampler::Sample>
UniformLightSampler::sample(float u, const float3& hit, const float3& ns) const
{
    if (mLights.empty())
    {
        return {};
    }

    std::size_t lightIdx = static_cast<std::size_t>(u * mLights.size());
    lightIdx = std::min(lightIdx, mLights.size() - 1);

    return Sample{ mLights[lightIdx], mPdf };
}
}
