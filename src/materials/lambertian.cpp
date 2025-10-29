#include "lambertian.h"

#include <utils/util.h>

#include <numbers>

namespace alpine {
std::optional<Bsdf::Sample>
Lambertian::sample(
    const float3& wo, const float2& u, const float3& baseColor) const
{
    auto [wi, pdf] = sampleCosineWeightedHemisphere(u);
    if (pdf == 0.0f)
    {
        return {};
    }

    float3 estimator = baseColor; // bsdf * cosTheta / pdf

    return Sample{ estimator, wi, pdf };
}

float3
Lambertian::computeBsdf(
    const float3& wo, const float3& wi, const float3& baseColor) const
{
    float3 bsdf = baseColor / std::numbers::pi_v<float>;

    return bsdf;
}

float
Lambertian::computePdf(const float3& wo, const float3& wi) const
{
    return std::max(cosTheta(wi), 0.0f) / std::numbers::pi_v<float>;
}
}
