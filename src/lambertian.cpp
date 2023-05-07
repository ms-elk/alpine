#include "lambertian.h"

#include "sampler.h"
#include "shape.h"
#include "util.h"

namespace alpine {
float3
Lambertian::evaluate(
    const float3& wo, const float3& wi, const IntersectionAttributes& isectAttr) const
{
    float3 albedo = mBaseColorTex ? mBaseColorTex->sample(isectAttr.uv).xyz() : mBaseColor;
    return albedo / PI;
}

Material::Sample
Lambertian::sample(
    const float3& wo, const float2& u, const IntersectionAttributes& isectAttr) const
{
    float pdf = 0.0f;
    float3 wiLocal = sampleCosineWeightedHemisphere(pdf, u);

    const auto toWorld = [&](const float3& v)
    {
        auto [ b1, b2 ] = getBasis(isectAttr.ns);
        return b1 * v.x + b2 * v.y + isectAttr.ns * v.z;
    };
    float3 wi = toWorld(wiLocal);
    float3 bsdf = evaluate(wo, wi, isectAttr);

    return { bsdf, wi, pdf };
}
}