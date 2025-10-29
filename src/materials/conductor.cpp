#include "conductor.h"

#include "microfacet.h"

#include <shapes/shape.h>
#include <utils/util.h>

namespace alpine {
Conductor::Conductor(const float2& alpha, const float3& baseColor,
    const std::shared_ptr<Texture4f>& baseColorTex,
    const std::shared_ptr<Texture4f>& normalTex)
    : mBaseColor(baseColor)
    , mBaseColorTex(baseColorTex)
    , mNormalTex(normalTex)
{
    mBsdf = std::make_unique<Microfacet>(alpha);
}

std::optional<Bsdf::Sample>
Conductor::sample(
    const float3& wo, const float2& u, const IntersectionAttributes& isectAttr) const
{
    float3 bc = getBaseColor(isectAttr.uv);
    auto sample = mBsdf->sample(wo, u, bc);

    return sample;
}

float3
Conductor::computeBsdf(
    const float3& wo, const float3& wi, const IntersectionAttributes& isectAttr) const
{
    float3 bc = getBaseColor(isectAttr.uv);
    float3 bsdf = mBsdf->computeBsdf(wo, wi, bc);

    return bsdf;
}

float
Conductor::computePdf(const float3& wo, const float3& wi) const
{
    return mBsdf->computePdf(wo, wi);
}

float3
Conductor::getBaseColor(const float2& uv) const
{
    return mBaseColorTex ? mBaseColorTex->sample(uv).xyz() : mBaseColor;
}

float3
Conductor::getNormal(const float2& uv) const
{
    return mNormalTex ? mNormalTex->sample(uv).xyz() : float3(0.0f, 0.0f, 1.0f);
}
}
