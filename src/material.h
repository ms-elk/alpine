#pragma once

#include "vector.h"

namespace alpine {
struct IntersectionAttributes;

class Material
{
public:
    Material() = default;
    virtual ~Material() = default;

    struct Sample {
        float3 estimator;
        float3 wi;
        float pdf;
    };

    virtual Sample sample(
        const float3& wo, const float2& u, const IntersectionAttributes& isectAttr) const = 0;

    virtual float3 evaluate(
        const float3& wo, const float3& wi, const IntersectionAttributes& isectAttr) const = 0;

    virtual float computePdf(const float3& wo, const float3& wi) const = 0;
};
}