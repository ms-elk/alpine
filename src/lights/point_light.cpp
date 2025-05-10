#include "point_light.h"

#include <utils/util.h>

#include <numbers>

namespace alpine {
PointLight::PointLight(const float3& intensity, const float3& position)
    : mIntensity(intensity)
{
    mPosition = position;
    mPower = mIntensity * 4.0f * std::numbers::pi_v<float>;
}

PointLight::PointLight(float power, const float3& color, const float3& position)
    : mPower(color * power)
{
    mPosition = position;
    mIntensity = mPower / (4.0f * std::numbers::pi_v<float>);
}

std::optional<Light::Sample>
PointLight::sample(const float2& u, const float3& hit) const
{
    float3 wiWorld = mPosition - hit;
    float distance2 = dot(wiWorld, wiWorld);
    if (distance2 == 0.0f)
    {
        return {};
    }

    float distance = std::sqrt(distance2);
    wiWorld /= distance;

    float3 emittedRadiance = mIntensity / distance2 * mScale;

    return Sample{ emittedRadiance, wiWorld, distance, 1.0f };
}
}
