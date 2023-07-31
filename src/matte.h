#pragma once

#include "material.h"
#include "texture.h"

#include <memory>

namespace alpine {
class Matte : public Material
{
public:
    Matte(const float3& baseColor, const std::shared_ptr<Texture<float4>>& baseColorTex)
        : mBaseColor(baseColor), mBaseColorTex(baseColorTex) {}

    Sample sample(
        const float3& wo, const float2& u, const IntersectionAttributes& isectAttr) const override;

    float3 computeBsdf(
        const float3& wo,const float3& wi, const IntersectionAttributes& isectAttr) const override;

    float computePdf(const float3& wo, const float3& wi) const override;

private:
    float3 getBaseColor(const float2& uv) const;

private:
    float3 mBaseColor;
    std::shared_ptr<Texture<float4>> mBaseColorTex;
};
}
