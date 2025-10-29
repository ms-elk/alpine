#pragma once

#include "bsdf.h"

namespace alpine {
class Lambertian final : public Bsdf
{
public:
    Lambertian() {}

    std::optional<Sample> sample(
        const float3& wo, const float2& u, const float3& baseColor) const override;

    float3 computeBsdf(
        const float3& wo, const float3& wi, const float3& baseColor) const override;

    float computePdf(const float3& wo, const float3& wi) const override;
};
}
