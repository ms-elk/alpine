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
        const float3 up(0.0f, 0.0f, 0.1f);
        float3 tan = cross(ng, up);

        if (length(tan) > 0.001f)
        {
            tan = normalize(tan);
            float3 bi = cross(tan, ng);
            return tan * v.x + bi * v.y + ng * v.z;
        }
        else
        {
            return ng.z >= 0.0f ? v : v * -1.0f;
        }
    };
    wi = toWorld(wiLocal);

    return evaluate(wo, wi);
}
}