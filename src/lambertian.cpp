#include "lambertian.h"

#include "sampler.h"
#include "shape.h"
#include "util.h"

namespace alpine {
float3
Lambertian::evaluate(
    const float3& wo, const float3& wi, const IntersectionAttributes& isectAttr) const
{
    float3 bc = getBaseColor(isectAttr.uv);
    float3 bsdf = bc / PI;

    return bsdf;
}

Material::Sample
Lambertian::sample(
    const float3& wo, const float2& u, const IntersectionAttributes& isectAttr) const
{
    auto [wi, pdf] = sampleCosineWeightedHemisphere(u);
    float3 bc = getBaseColor(isectAttr.uv);
    float3 estimator = bc; // bsdf * cosTheta / pdf

    return { estimator, wi, pdf };
}

float3
Lambertian::getBaseColor(const float2& uv) const
{
    return mBaseColorTex ? mBaseColorTex->sample(uv).xyz() : mBaseColor;
}

}
