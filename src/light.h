#pragma once

#include "vector.h"

namespace alpine {
struct IntersectionAttributes;

class Light
{
public:
    Light() = default;
    virtual ~Light() = default;

    struct Sample {
        float3 emission;
        float3 wiWorld;
        float distance;
        float pdf;
    };

    virtual Sample sample(const float2& u, const float3& hit) const = 0;

    virtual std::pair<float /* pdf */, float /* distance */>
        computePdf(const float3& hit, const float3& wiWorld) const = 0;

    virtual float3 getEmission() const = 0;
};
}
