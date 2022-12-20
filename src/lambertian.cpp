#include "lambertian.h"

#include "sampler.h"
#include "util.h"

namespace alpine {
Vector3f
Lambertian::evaluate(const Vector3f& wo, const Vector3f& wi) const
{
    return mAlbedo / PI;
}

Vector3f
Lambertian::sample(
    const Vector3f& wo,
    const Vector3f& ng,
    const Vector2f& u,
    Vector3f& wi,
    float& pdf) const
{
    wi = sampleCosineWeightedHemisphere(pdf, u);
    wi = transformBasis(wi, ng);

    return evaluate(wo, wi);
}
}