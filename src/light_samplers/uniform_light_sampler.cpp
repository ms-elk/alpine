#include "uniform_light_sampler.h"

namespace alpine {
UniformLightSampler::UniformLightSampler(const std::vector<std::shared_ptr<Light>>& lights)
    : mLights(lights)
{
    mPdf = !mLights.empty() ? 1.0f / mLights.size() : 0.0f;
}

LightSampler::Sample
UniformLightSampler::sample(float u) const
{
    if (mLights.empty())
    {
        return { nullptr, 0.0f };
    }

    std::size_t lightIdx = static_cast<std::size_t>(u * mLights.size());
    lightIdx = std::min(lightIdx, mLights.size() - 1);

    return { mLights[lightIdx].get(), mPdf };
}
}
