#pragma once

#include "vector.h"

namespace alpine {
struct IntersectionAttributes;

class Material
{
public:
    Material() {}
    virtual ~Material() {};

    virtual float3 evaluate(
        const float3& wo, const float3& wi, const IntersectionAttributes& isectAttr) const = 0;

    struct Sample {
        float3 bsdf;
        float3 wi;
        float pdf;
    };

    virtual Sample sample(
        const float3& wo, const float2& u, const IntersectionAttributes& isectAttr) const = 0;
};
}