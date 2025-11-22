#include "dielectric.h"

#include "lambertian.h"
#include "microfacet.h"

#include <shapes/shape.h>
#include <utils/util.h>

namespace {
constexpr float REFLECTANCE = 0.5f;
constexpr float F0 = 0.16f * REFLECTANCE * REFLECTANCE;
}

namespace alpine {
Dielectric::Dielectric(const float2& alpha, const float3& baseColor,
    const std::shared_ptr<Texture4f>& baseColorTex,
    const std::shared_ptr<Texture4f>& normalTex)
    : mBaseColor(baseColor)
    , mBaseColorTex(baseColorTex)
    , mNormalTex(normalTex)
{
    mBsdfs.reserve(2);
    mBsdfs.push_back(std::make_unique<Lambertian>());
    if (dot(alpha, alpha) < 1.0f) {
        mBsdfs.push_back(std::make_unique<Microfacet>(alpha, F0));
    }
}

std::optional<Bsdf::Sample>
Dielectric::sample(
    const float3& wo, const float2& u, const IntersectionAttributes& isectAttr) const
{
    float2 uu = u;
    uint8_t bsdfIdx = static_cast<uint32_t>(uu[0] * mBsdfs.size());
    
    uu[0] = uu[0] * mBsdfs.size() - static_cast<float>(bsdfIdx);

    float3 bc = getBaseColor(isectAttr.uv);
    auto os = mBsdfs[bsdfIdx]->sample(wo, uu, bc);

    if (!os.has_value())
    {
        return {};
    }

    auto sample = os.value();
    float cosTerm = cosTheta(sample.wi);
    if (cosTerm <= 0.0f)
    {
        return {};
    }

    float3 bsdf = sample.estimator * sample.pdf / cosTerm;
    for (uint8_t i = 0; i < mBsdfs.size(); ++i)
    {
        if (i != bsdfIdx)
        {
            sample.pdf += mBsdfs[i]->computePdf(wo, sample.wi);
            bsdf += mBsdfs[i]->computeBsdf(wo, sample.wi, bc);
        }
    }

    sample.pdf /= static_cast<float>(mBsdfs.size());
    sample.estimator = bsdf * cosTerm / sample.pdf;

    return sample;
}

float3
Dielectric::computeBsdf(
    const float3& wo, const float3& wi, const IntersectionAttributes& isectAttr) const
{
    float3 bsdfSum(0.0f);
    float3 bc = getBaseColor(isectAttr.uv);
    for (const auto& bsdf : mBsdfs)
    {
        bsdfSum += bsdf->computeBsdf(wo, wi, bc);
    }

    return bsdfSum;
}

float
Dielectric::computePdf(const float3& wo, const float3& wi) const
{
    float pdfSum = 0.0f;
    for (const auto& bsdf : mBsdfs)
    {
        pdfSum += bsdf->computePdf(wo, wi);
    }

    float pdf = pdfSum / static_cast<float>(mBsdfs.size());

    return pdf;
}

float3
Dielectric::getBaseColor(const float2& uv) const
{
    return mBaseColorTex ? mBaseColorTex->sample(uv).xyz() : mBaseColor;
}

float3
Dielectric::getNormal(const float2& uv) const
{
    return mNormalTex ? mNormalTex->sample(uv).xyz() : float3(0.0f, 0.0f, 1.0f);
}
}
