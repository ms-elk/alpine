#include "microfacet.h"

#include "sampler.h"
#include "shape.h"
#include "util.h"

#define VNDF_SAMPLEING

namespace alpine {
float3
Microfacet::evaluate(
    const float3& wo, const float3& wi, const IntersectionAttributes& isectAttr) const
{
    float cosThetaWo = std::abs(cosTheta(wo));
    float cosThetaWi = std::abs(cosTheta(wi));
    if (cosThetaWo == 0.0f || cosThetaWi == 0.0f)
    {
        return float3(0.0f);
    }

    float3 wh = wo + wi;
    float whLength = length(wh);
    if (whLength == 0.0f)
    {
        return float3(0.0f);
    }
    wh /= whLength;

    float3 bc = getBaseColor(isectAttr.uv);
    float d = computeDistribution(wh);
    float g2 = computeMaskingShadowing(wo, wi);
    float3 bsdf = bc * d * g2 / (4.0f * cosThetaWo * cosThetaWi);

    return bsdf;
}

Material::Sample
Microfacet::sample(
    const float3& wo, const float2& u, const IntersectionAttributes& isectAttr) const
{
#ifdef VNDF_SAMPLEING
    float3 woh = normalize(float3(mAlpha.x * wo.x, mAlpha.y * wo.y, wo.z));

    float2 t = sampleConcentricDisk(u);
    float s = 0.5f * (1.0f + woh.z);
    t.y = (1.0f - s) * std::sqrt(1.0f - t.x * t.x) + s * t.y;
    float3 nhh(t.x, t.y, std::sqrt(std::max(0.0f, 1.0f - t.x * t.x - t.y * t.y)));

    auto [b1, b2] = getBasis(woh);
    float3 nh = b1 * nhh.x + b2 * nhh.y + woh * nhh.z;

    float3 wh = normalize(float3(mAlpha.x * nh.x, mAlpha.y * nh.y, std::max(0.0f, nh.z)));
    if (!isSameHemisphere(wo, wh))
    {
        wh = -wh;
    }

    float3 wi = reflect(wo, wh);

    if (isSameHemisphere(wo, wi))
    {
        float3 bc = getBaseColor(isectAttr.uv);
        float g1 = 1.0f / (1.0f + lambda(wo));
        float g2 = computeMaskingShadowing(wo, wi);
        float3 estimator = bc * g2 / g1;

        float d = computeDistribution(wh);
        float pdf = g1 * std::max(0.0f, dot(wo, wh)) * d / wo.z;
        pdf /= (4.0f * dot(wo, wh));

        return { estimator, wi, pdf };
    }
    else
    {
        return { 0.0f, float3(0.0f), 0.0f };
    }

#else
    auto [wi, pdf] = sampleCosineWeightedHemisphere(u);

    if (isSameHemisphere(wo, wi) && pdf > 0.0f)
    {
        float3 estimator = evaluate(wo, wi, isectAttr) * std::abs(cosTheta(wi)) / pdf;
        return { estimator, wi, pdf };
    }
    else
    {
        return { 0.0f, float3(0.0f), 0.0f };
    }
#endif
}

float
Microfacet::computeDistribution(const float3& wh) const
{
    float cos4Theta = cos2Theta(wh) * cos2Theta(wh);
    float cos2Phi = cosPhi(wh) * cosPhi(wh);
    float sin2Phi = sinPhi(wh) * sinPhi(wh);
    float term = 1.0f + tan2Theta(wh) * (cos2Phi / (mAlpha.x * mAlpha.x) + sin2Phi / (mAlpha.y * mAlpha.y));
    float invD = PI * mAlpha.x * mAlpha.y * cos4Theta * term * term;

    return invD != 0.0f ? 1.0f / invD : 0.0f;
}

float
Microfacet::lambda(const float3& v) const
{
    float alpha = std::sqrt(cosPhi(v) * cosPhi(v) * mAlpha.x * mAlpha.x
        + sinPhi(v) * sinPhi(v) * mAlpha.y * mAlpha.y);
    return (-1.0f + std::sqrt(1.0f + alpha * alpha * tan2Theta(v))) / 2.0f;
}

float
Microfacet::computeMaskingShadowing(const float3& wo, const float3& wi) const
{
    return 1.0f / (1.0f + lambda(wo) + lambda(wi));
}

float3
Microfacet::getBaseColor(const float2& uv) const
{
    return mBaseColorTex ? mBaseColorTex->sample(uv).xyz() : mBaseColor;
}
}