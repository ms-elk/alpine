#include "disk_light.h"

#include "shape.h"
#include "util.h"

namespace alpine {
DiskLight::DiskLight(const float3& emission, const float3& position, const float3& normal, float radius)
    : mEmission(emission), mPosition(position), mNormal(normal), mRadius(radius)
{
    mArea = mRadius * mRadius * PI;
}

DiskLight::Sample
DiskLight::sample(const float2& u, const float3& hit) const
{
    float2 diskSample = sampleConcentricDisk(u);
    float3 lightSample = mPosition + float3(diskSample.x, 0.0f, diskSample.y) * mRadius;
    float3 wiWorld = lightSample - hit;
    float distance = length(wiWorld);

    if (distance == 0.0f)
    {
        return { float3(0.0f), float3(0.0f), 0.0f, 0.0f };
    }

    wiWorld /= distance;
    float cosTerm = std::max(0.0f, dot(-wiWorld, mNormal));
    if (cosTerm == 0.0f)
    {
        return { float3(0.0f), float3(0.0f), 0.0f, 0.0f };
    }

    float pdf = (distance * distance) / (mArea * cosTerm);

    return { mEmission, wiWorld, distance, pdf };
}

std::pair<float /* pdf */, float /* distance */>
DiskLight::computePdf(const float3& hit, const float3& wiWorld) const
{
    float cosTerm = std::max(0.0f, dot(-wiWorld, mNormal));
    if (cosTerm == 0.0f)
    {
        return { 0.0f, 0.0f };
    }

    float distance = dot(hit - mPosition, mNormal) / cosTerm;
    if (distance < 0.0f)
    {
        return { 0.0f, 0.0f };
    }

    float3 lightSample = hit + wiWorld * distance;
    bool inside = dot(lightSample - mPosition, lightSample - mPosition) < mRadius * mRadius;
    if (inside && cosTerm > 0.0f)
    {
        float pdf = (distance * distance) / (mArea * cosTerm);
        return { pdf, distance };
    }
    else
    {
        return { 0.0f, 0.0f };
    }
}
}
