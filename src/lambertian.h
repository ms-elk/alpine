#pragma once

#include "material.h"

namespace alpine {
class Lambertian : public Material
{
public:
    Lambertian (const float3& albedo) : mAlbedo(albedo) {}
    virtual ~Lambertian() {}

    virtual float3 evaluate(const float3& wo,const float3& wi) const override;

    virtual float3 sample(
        const float3& wo,
        const float3& ng,
        const float2& u,
        float3& wi,
        float& pdf) const override;

private:
    float3 mAlbedo;
};
}