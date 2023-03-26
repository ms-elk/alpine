#include "lambertian.h"

#include "sampler.h"
#include "util.h"

namespace alpine {
float3
Lambertian::evaluate(const float3& wo, const float3& wi) const
{
    return mAlbedo / PI;
}

float3
Lambertian::sample(
    const float3& wo,
    const float3& ng,
    const float2& u,
    float3& wi,
    float& pdf) const
{
    float3 wiLocal = sampleCosineWeightedHemisphere(pdf, u);

    const auto toWorld = [&](const float3& v)
    {
        auto [ b1, b2 ] = getBasis(ng);
        return b1 * v.x + b2 * v.y + ng * v.z;
    };
    wi = toWorld(wiLocal);

    return evaluate(wo, wi);
}
}