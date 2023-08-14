#pragma once

#include "light.h"
#include "vector.h"

namespace alpine {
struct IntersectionAttributes;

class PointLight : public Light
{
public:
    PointLight(const float3& intensity, const float3& position);

    Sample sample(const float2& u, const float3& hit) const override;

    std::pair<float /* pdf */, float /* distance */>
        computePdf(const float3& hit, const float3& wiWorld) const override
    {
        return { 0.0f, 0.0f };
    }

    float3 getEmission() const override { return 0.0f; }

    float3 getPower() const override;

private:
    float3 mIntensity;
    float3 mPosition;
};
}
