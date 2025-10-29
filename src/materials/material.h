#pragma once

#include "bsdf.h"

#include <math/vector.h>

#include <optional>

namespace alpine {
struct IntersectionAttributes;

class Material
{
public:
    Material() = default;
    virtual ~Material() = default;

    virtual std::optional<Bsdf::Sample> sample(
        const float3& wo, const float2& u, const IntersectionAttributes& isectAttr) const = 0;

    virtual float3 computeBsdf(
        const float3& wo, const float3& wi, const IntersectionAttributes& isectAttr) const = 0;

    virtual float computePdf(const float3& wo, const float3& wi) const = 0;

    virtual float3 getBaseColor(const float2& uv) const = 0;

    virtual float3 getNormal(const float2& uv) const = 0;
};
}
