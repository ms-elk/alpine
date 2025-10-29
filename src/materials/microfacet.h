#pragma once

#include "bsdf.h"

namespace alpine {
class Microfacet final : public Bsdf
{
public:
    Microfacet(const float2& alpha, float f0 = -1.0f)
        : mAlpha({ std::max(0.001f, alpha.x), std::max(0.001f, alpha.y) })
        , mF0(f0)
    {}

    std::optional<Sample> sample(
        const float3& wo, const float2& u, const float3& baseColor) const override;

    float3 computeBsdf(
        const float3& wo, const float3& wi, const float3& baseColor) const override;

    float computePdf(const float3& wo, const float3& wi) const override;

private:
    float computeDistribution(const float3& wh) const;

    float lambda(const float3& v) const;

    float computeMaskingShadowing(const float3& wo, const float3& wi) const;

    bool isMetallic() const { return mF0 < 0.0f; }

private:
    float2 mAlpha;
    float mF0;
};
}
