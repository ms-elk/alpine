#pragma once

#include "material.h"

#include <texture.h>

#include <array>
#include <memory>

namespace alpine {
class Bsdf;

class Dielectric final : public Material
{
public:
    Dielectric(
        const float2& alpha,
        const float3& baseColor,
        const std::shared_ptr<Texture4f>& baseColorTex,
        const std::shared_ptr<Texture4f>& normalTex);

    std::optional<Bsdf::Sample> sample(
        const float3& wo, const float2& u, const IntersectionAttributes& isectAttr) const override;

    float3 computeBsdf(
        const float3& wo, const float3& wi, const IntersectionAttributes& isectAttr) const override;

    float computePdf(const float3& wo, const float3& wi) const override;

    float3 getBaseColor(const float2& uv) const override;

    float3 getNormal(const float2& uv) const override;

private:
    std::array<std::unique_ptr<Bsdf>, 2> mBsdfs;
    float3 mBaseColor;
    std::shared_ptr<Texture4f> mBaseColorTex;
    std::shared_ptr<Texture4f> mNormalTex;
};
}
