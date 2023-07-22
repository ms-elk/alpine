﻿#pragma once

#include "material.h"
#include "texture.h"

#include <memory>

namespace alpine {
class Lambertian : public Material
{
public:
    Lambertian(const float3& baseColor, const std::shared_ptr<Texture<float4>>& baseColorTex)
        : mBaseColor(baseColor), mBaseColorTex(baseColorTex) {}
    virtual ~Lambertian() = default;

    virtual Sample sample(
        const float3& wo, const float2& u, const IntersectionAttributes& isectAttr) const override;

    virtual float3 evaluate(
        const float3& wo,const float3& wi, const IntersectionAttributes& isectAttr) const override;

    virtual float computePdf(const float3& wo, const float3& wi) const override;

private:
    float3 getBaseColor(const float2& uv) const;

private:
    float3 mBaseColor;
    std::shared_ptr<Texture<float4>> mBaseColorTex;
};
}
