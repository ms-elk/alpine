#pragma once

#include <math/vector.h>
#include <texture.h>

#include <optional>

namespace alpine {
struct IntersectionAttributes;

class Material
{
public:
    Material() = default;
    virtual ~Material() = default;

    struct Sample {
        float3 estimator; // bsdf * cosTerm / pdf
        float3 wi;
        float pdf;
    };

    virtual std::optional<Sample> sample(
        const float3& wo, const float2& u, const IntersectionAttributes& isectAttr) const = 0;

    virtual float3 computeBsdf(
        const float3& wo, const float3& wi, const IntersectionAttributes& isectAttr) const = 0;

    virtual float computePdf(const float3& wo, const float3& wi) const = 0;

    virtual float3 getBaseColor(const float2& uv) const = 0;

    virtual float3 getNormal(const float2& uv) const = 0;
};
}
