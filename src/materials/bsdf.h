#pragma once

#include <math/vector.h>

#include <optional>

namespace alpine {

class Bsdf
{
public:
    Bsdf() = default;
    virtual ~Bsdf() = default;

    struct Sample {
        float3 estimator; // bsdf * cosTerm / pdf
        float3 wi;
        float pdf;
    };

    virtual std::optional<Sample> sample(
        const float3& wo, const float2& u, const float3& baseColor) const = 0;

    virtual float3 computeBsdf(
        const float3& wo, const float3& wi, const float3& baseColor) const = 0;

    virtual float computePdf(const float3& wo, const float3& wi) const = 0;
};
}
