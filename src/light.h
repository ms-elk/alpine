#pragma once

#include "vector.h"

namespace alpine {
struct IntersectionAttributes;

class Light
{
public:
    Light(const float3& emission, const float3& position, const float3& normal, float radius);

    struct Sample {
        float3 emission;
        float3 wi;
        float distance;
        float pdf;
    };

    Sample sample(const float2& u, const float3& hit) const;

    std::pair<float /* pdf */, float /* distance */>
        computePdf(const float3& hit, const float3& wi) const;

    float3 getEmission() const { return mEmission; }

private:
    float3 mEmission;
    float3 mPosition;
    float3 mNormal;
    float mRadius;
    float mArea;
};
}
