#include "metal.h"

#include "sampler.h"
#include "shapes/shape.h"
#include "utils/util.h"

namespace alpine {
std::optional<Material::Sample>
Metal::sample(
    const float3& wo, const float2& u, const IntersectionAttributes& isectAttr) const
{
    if (mUseVndfSampling)
    {
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
        if (!isSameHemisphere(wo, wi))
        {
            return {};
        }

        float3 bc = getBaseColor(isectAttr.uv);
        float3 f = schlickFresnel(bc, wi, wh);
        float g1 = 1.0f / (1.0f + lambda(wo));
        float g2 = computeMaskingShadowing(wo, wi);
        float d = computeDistribution(wh);

        float3 estimator = f * g2 / g1;
        float pdf = d / (4.0f * dot(wo, wh));

        return Sample{ estimator, wi, pdf };
    }
    else
    {
        auto [wi, pdf] = sampleCosineWeightedHemisphere(u);
        if (!isSameHemisphere(wo, wi) || pdf == 0.0f)
        {
            return {};
        }

        float3 estimator = computeBsdf(wo, wi, isectAttr) * std::abs(cosTheta(wi)) / pdf;

        return Sample{ estimator, wi, pdf };
    }
}

float3
Metal::computeBsdf(
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
    float3 f = schlickFresnel(bc, wi, wh);
    float d = computeDistribution(wh);
    float g2 = computeMaskingShadowing(wo, wi);
    float3 bsdf = f * d * g2 / (4.0f * cosThetaWo * cosThetaWi);

    return bsdf;
}

float
Metal::computePdf(const float3& wo, const float3& wi) const
{
    if (mUseVndfSampling)
    {
        float3 wh = wo + wi;
        float whLength = length(wh);
        if (whLength == 0.0f)
        {
            return 0.0f;
        }
        wh /= whLength;

        float d = computeDistribution(wh);
        float pdf = d / (4.0f * dot(wo, wh));

        return pdf;
    }
    else
    {
        return std::max(cosTheta(wi), 0.0f) / PI;
    }
}

float
Metal::computeDistribution(const float3& wh) const
{
    float cos4Theta = cos2Theta(wh) * cos2Theta(wh);
    float cos2Phi = cosPhi(wh) * cosPhi(wh);
    float sin2Phi = sinPhi(wh) * sinPhi(wh);
    float term = 1.0f + tan2Theta(wh) * (cos2Phi / (mAlpha.x * mAlpha.x) + sin2Phi / (mAlpha.y * mAlpha.y));
    float invD = PI * mAlpha.x * mAlpha.y * cos4Theta * term * term;

    return invD != 0.0f ? 1.0f / invD : 0.0f;
}

float
Metal::lambda(const float3& v) const
{
    float alpha = std::sqrt(cosPhi(v) * cosPhi(v) * mAlpha.x * mAlpha.x
        + sinPhi(v) * sinPhi(v) * mAlpha.y * mAlpha.y);
    return (-1.0f + std::sqrt(1.0f + alpha * alpha * tan2Theta(v))) / 2.0f;
}

float
Metal::computeMaskingShadowing(const float3& wo, const float3& wi) const
{
    return 1.0f / (1.0f + lambda(wo) + lambda(wi));
}

float3
Metal::getBaseColor(const float2& uv) const
{
    return mBaseColorTex ? mBaseColorTex->sample(uv).xyz() : mBaseColor;
}

float3
Metal::getNormal(const float2& uv) const
{
    return mNormalTex ? mNormalTex->sample(uv).xyz() : float3(0.0f, 0.0f, 1.0f);
}
}
