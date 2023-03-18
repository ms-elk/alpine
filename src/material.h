#pragma once

#include "vector.h"

namespace alpine {
namespace kernel {
struct Intersection;
}

class Material
{
public:
    Material() {}
    virtual ~Material() {};

    // TODO: implement BSDF class, and remove ng
    virtual float3 evaluate(const float3& wo, const float3& wi) const = 0;

    virtual float3 sample(
        const float3& wo,
        const float3& ng, // tmp
        const float2& u,
        float3& wi,
        float& pdf) const = 0;
};
}