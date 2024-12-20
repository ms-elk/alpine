#include "util.h"

#include <numbers>

namespace alpine {
float2
sampleConcentricDisk(const float2& u)
{
    // map uniform random numbers to $[-1,1]^2$
    float2 uOffset(2.0f * u.x - 1.0f, 2.0f * u.y - 1.0f);

    // handle degeneracy at the origin
    if (uOffset.x == 0 && uOffset.y == 0)
    {
        return float2(0.0f);
    }

    // apply concentric mapping to point
    float theta, r;
    if (std::abs(uOffset.x) > std::abs(uOffset.y))
    {
        r = uOffset.x;
        theta = 0.25f * std::numbers::pi_v<float> * (uOffset.y / uOffset.x);
    }
    else
    {
        r = uOffset.y;
        theta = 0.5f * std::numbers::pi_v<float> * (1.0f - 0.5f * (uOffset.x / uOffset.y));
    }
    return float2(r * std::cos(theta), r * std::sin(theta));
}

std::pair<float3 /* dir */, float /* pdf */>
sampleCosineWeightedHemisphere(const float2& u)
{
    float2 d = sampleConcentricDisk(u);
    float z = std::sqrt(std::max(0.0f, 1.0f - d.x * d.x - d.y * d.y));
    float3 dir = normalize(float3(d.x, d.y, z));
    float pdf = z / std::numbers::pi_v<float>;
    return { dir, pdf };
}

std::pair<float3 /* dir*/, float /* pdf */>
sampleHemisphere(const float2& u)
{
    float z = u.x;
    float r = std::sqrt(std::max(0.0f, 1.0f - z * z));
    float phi = 2.0f * std::numbers::pi_v<float> * u.y;
    float3 dir(r * std::cos(phi), r * std::sin(phi), z);
    return { dir, 1.0f / (2.0f * std::numbers::pi_v<float>) };
}

}
