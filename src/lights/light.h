#pragma once

#include "math/vector.h"

#include <alpine/light.h>

namespace alpine {
struct IntersectionAttributes;

class Light : public api::Light
{
public:
    Light() = default;
    virtual ~Light() = default;

    void setPosition(const float position[3]) override { mPosition = float3(position); }

    void setScale(float scale) override { mScale = scale; }

    struct Sample {
        float3 emittedRadiance;
        float3 wiWorld;
        float distance;
        float pdf;
    };

    virtual Sample sample(const float2& u, const float3& hit) const = 0;

    virtual std::pair<float /* pdf */, float /* distance */>
        computePdf(const float3& hit, const float3& wiWorld) const = 0;

    virtual float3 getEmittedRadiance() const = 0;

    virtual float3 getPower() const = 0;

    virtual bool isDelta() const = 0;

protected:
    float3 mPosition;
    float mScale = 1.0f;
};
}
