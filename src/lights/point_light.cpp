#include "point_light.h"

#include "utils/util.h"

namespace alpine {
PointLight::PointLight(const float3& intensity, const float3& position)
    : mIntensity(intensity)
{
    mPosition = position;
}

PointLight::Sample
PointLight::sample(const float2& u, const float3& hit) const
{
    float3 wiWorld = mPosition - hit;
    float distance2 = dot(wiWorld, wiWorld);
    if (distance2 == 0.0f)
    {
        return { float3(0.0f), float3(0.0f), 0.0f, 0.0f };
    }

    float distance = std::sqrt(distance2);
    wiWorld /= distance;

    float3 emission = mIntensity / distance2 * mScale;

    return { emission, wiWorld, distance, 1.0f };
}

float3
PointLight::getPower() const
{
    return mIntensity * 4.0f * PI * mScale;
}
}
