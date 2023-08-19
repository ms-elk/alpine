#pragma once

#include "material.h"
#include "texture.h"

#include <memory>

namespace alpine {
class Metal : public Material
{
public:
    Metal(const float2& alpha, const float3& baseColor,
        const std::shared_ptr<Texture4f>& baseColorTex,
        const std::shared_ptr<Texture4f>& normalTex,
        bool useVndfSampling = true)
        : mAlpha({ std::max(0.001f, alpha.x), std::max(0.001f, alpha.y) })
        , mBaseColor(baseColor)
        , mBaseColorTex(baseColorTex)
        , mNormalTex(normalTex)
        , mUseVndfSampling(useVndfSampling) {};

    Sample sample(
        const float3& wo, const float2& u, const IntersectionAttributes& isectAttr) const override;

    float3 computeBsdf(
        const float3& wo, const float3& wi, const IntersectionAttributes& isectAttr) const override;

    float computePdf(const float3& wo, const float3& wi) const override;

    const Texture4f* getNormalTex() const override { return mNormalTex.get(); }

private:
    float computeDistribution(const float3& wh) const;

    float lambda(const float3& v) const;

    float computeMaskingShadowing(const float3& wo, const float3& wi) const;

    float3 getBaseColor(const float2& uv) const;

private:
    float2 mAlpha;
    float3 mBaseColor;
    std::shared_ptr<Texture4f> mBaseColorTex;
    std::shared_ptr<Texture4f> mNormalTex;
    bool mUseVndfSampling;
};
}
