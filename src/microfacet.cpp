#include "microfacet.h"

#include "sampler.h"
#include "shape.h"
#include "util.h"

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
    float d = computeDistribution(wh);
    float term = d / (4.0f * cosThetaWo * cosThetaWi);

    if (term > 1.0f)
    {
        printf("d: %f\n", term);
    }

    float3 albedo = mBaseColorTex ? mBaseColorTex->sample(isectAttr.uv).xyz() : mBaseColor;
    return albedo * term;
}

Material::Sample
Microfacet::sample(
    const float3& wo, const float2& u, const IntersectionAttributes& isectAttr) const
{
#if 0
    float3 woh = normalize(float3(mAlpha.x * wo.x, mAlpha.y * wo.y, wo.z));

    float2 t = sampleConcentricDisk(u);
    float s = 0.5f * (1.0f + woh.z);
    t.y = (1.0f - s) * std::sqrt(1.0f - t.x * t.x) + s * t.y;
    float3 nhLocal(t.x, t.y, std::sqrt(std::max(0.0f, 1.0f - t.x * t.x - t.y * t.y)));

    const auto toWorld = [&](const float3& v)
    {
        auto [b1, b2] = getBasis(woh);
        return b1 * v.x + b2 * v.y + woh * v.z;
    };
    float3 whh = toWorld(nhLocal);
    float3 wh = normalize(float3(mAlpha.x * whh.x, mAlpha.y * whh.y, std::max(0.0f, whh.z)));
    float3 wi = reflect(wo, whh);

    float d = computeDistribution(wh);
    float pdf = d / (4.0f * dot(wo, wh));

    float3 bsdf = evaluate(wo, wi, isectAttr);
#else
    auto [wiLocal, pdf] = sampleCosineWeightedHemisphere(u);

    const auto toWorld = [&](const float3& v)
    {
        auto [b1, b2] = getBasis(isectAttr.ns);
        return b1 * v.x + b2 * v.y + isectAttr.ns * v.z;
    };
    float3 wi = toWorld(wiLocal);
    float3 bsdf = evaluate(wo, wi, isectAttr);
#endif

    return { bsdf, wi, pdf };
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
}
