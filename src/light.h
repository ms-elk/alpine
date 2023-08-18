#pragma once

#include "vector.h"

#include <i_light.h>

namespace alpine {
struct IntersectionAttributes;

class Light : public ILight
{
public:
    Light() = default;
    virtual ~Light() = default;

    void enable(bool enabled) override { mEnabled = enabled; }

    bool isEnabled() const override { return mEnabled; }

    void setPosition(const float position[3]) override { mPosition = float3(position); }

    void setScale(float scale) override { mScale = scale; }

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

    virtual float3 getPower() const = 0;

protected:
    bool mEnabled = true;
    float3 mPosition;
    float mScale = 1.0f;
};
}
