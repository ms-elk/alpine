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
    wi = sampleCosineWeightedHemisphere(pdf, u);
    wi = transformBasis(wi, ng);

    return evaluate(wo, wi);
}
}