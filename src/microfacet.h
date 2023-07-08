#pragma once

#include "material.h"
#include "texture.h"

#include <memory>

namespace alpine {
class Microfacet : public Material
{
public:
    Microfacet(const float2& alpha,
        const float3& baseColor, const std::shared_ptr<Texture<float4>>& baseColorTex)
        : mAlpha({ std::max(0.001f, alpha.x), std::max(0.001f, alpha.y) })
        , mBaseColor(baseColor)
        , mBaseColorTex(baseColorTex) {};
    virtual ~Microfacet() = default;

    virtual float3 evaluate(
        const float3& wo, const float3& wi, const IntersectionAttributes& isectAttr) const override;

    virtual Sample sample(
        const float3& wo, const float2& u, const IntersectionAttributes& isectAttr) const override;

private:
    float computeDistribution(const float3& wh) const;

    float lambda(const float3& v) const;

    float computeMaskingShadowing(const float3& wo, const float3& wi) const;

    float3 getBaseColor(const float2& uv) const;

private:
    float2 mAlpha;
    float3 mBaseColor;
    std::shared_ptr<Texture<float4>> mBaseColorTex;
};
}
