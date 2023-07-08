#include "light.h"

#include "shape.h"
#include "util.h"

namespace alpine {
Light::Light(const float3& emission, const float3& position, const float3& normal, float radius)
    : mEmission(emission), mPosition(position), mNormal(normal), mRadius(radius)
{
    mArea = radius * radius * PI;
}

Light::Sample
Light::sample(const float2& u, const float3& hit) const
{
    float2 diskSample = sampleConcentricDisk(u);
    float3 lightSample = mPosition + float3(diskSample.x, 0.0f, diskSample.y) * mRadius;
    float3 wi = lightSample - hit;
    float distance = length(wi);

    if (distance == 0.0f)
    {
        return { float3(0.0f), float3(0.0f), 0.0f, 0.0f };
    }

    wi /= distance;
    float cosTerm = std::abs(dot(wi, mNormal)); // TODO
    float pdf = (distance * distance) / (mArea * cosTerm);

    return { mEmission, wi, distance, pdf };
}

std::pair<float /* pdf */, float /* distance */>
Light::computePdf(const float3& hit, const float3& wi) const
{
    if (wi.y == 0.0f)
    {
        return { 0.0f, 0.0f };
    }

    float distance = (mPosition.y - hit.y) / wi.y;
    if (distance < 0.0f)
    {
        return { 0.0f, 0.0f };
    }

    float3 lightSample = hit + wi * distance;
    bool inside = dot(lightSample - mPosition, lightSample - mPosition) < mRadius * mRadius;
    if (inside)
    {
        float cosTerm = std::abs(dot(wi, mNormal)); // TODO
        float pdf = (distance * distance) / (mArea * cosTerm);
        return { pdf, distance };
    }
    else
    {
        return { 0.0f, 0.0f };
    }
}
}
