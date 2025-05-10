#pragma once

#include "light.h"

#include <math/vector.h>

namespace alpine {
struct IntersectionAttributes;

class PointLight final : public Light
{
public:
    PointLight(const float3& intensity, const float3& position);
    PointLight(float power, const float3& color, const float3& position);

    std::optional<Sample> sample(const float2& u, const float3& hit) const override;

    std::pair<float /* pdf */, float /* distance */>
        computePdf(const float3& hit, const float3& wiWorld) const override
    {
        return { 0.0f, 0.0f };
    }

    float3 getEmittedRadiance() const override { return 0.0f; }

    float3 getPower() const override { return mPower * mScale; }

    bool isDelta() const override { return true; }

    BoundingBox getBoundingBox() const override { return{ mPosition, mPosition }; }

private:
    float3 mIntensity;
    float3 mPower;
};
}
