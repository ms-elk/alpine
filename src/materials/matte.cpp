#include "matte.h"

#include <sampler.h>
#include <shapes/shape.h>
#include <utils/util.h>

#include <numbers>

namespace alpine {
std::optional<Material::Sample>
Matte::sample(
    const float3& wo, const float2& u, const IntersectionAttributes& isectAttr) const
{
    auto [wi, pdf] = sampleCosineWeightedHemisphere(u);
    if (pdf == 0.0f)
    {
        return {};
    }

    float3 bc = getBaseColor(isectAttr.uv);
    float3 estimator = bc; // bsdf * cosTheta / pdf

    return Sample{ estimator, wi, pdf };
}

float3
Matte::computeBsdf(
    const float3& wo, const float3& wi, const IntersectionAttributes& isectAttr) const
{
    float3 bc = getBaseColor(isectAttr.uv);
    float3 bsdf = bc / std::numbers::pi_v<float>;

    return bsdf;
}

float
Matte::computePdf(const float3& wo, const float3& wi) const
{
    return std::max(cosTheta(wi), 0.0f) / std::numbers::pi_v<float>;
}

float3
Matte::getBaseColor(const float2& uv) const
{
    return mBaseColorTex ? mBaseColorTex->sample(uv).xyz() : mBaseColor;
}

float3
Matte::getNormal(const float2& uv) const
{
    return mNormalTex ? mNormalTex->sample(uv).xyz() : float3(0.0f, 0.0f, 1.0f);
}
}
