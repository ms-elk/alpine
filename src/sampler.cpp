#include "sampler.h"

#include "util.h"
#include <algorithm>

namespace alpine {
void
Sampler::reset(uint32_t seed)
{
    for (uint32_t i = 1; i <= 4; ++i)
    {
        mSeed[i - 1] = seed = 1812433253U * (seed ^ (seed >> 30)) + i;
    }
}

float
Sampler::get1D()
{
    uint32_t rng = next();
    double value = static_cast<double>(rng)
        / static_cast<double>(std::numeric_limits<uint32_t>::max());
    return static_cast<float>(value);
}

float2
Sampler::get2D()
{
    return { get1D(), get1D() };
}

uint32_t
Sampler::next()
{
    uint32_t t = mSeed[3];
    uint32_t s = mSeed[0];
    mSeed[3] = mSeed[2];
    mSeed[2] = mSeed[1];
    mSeed[1] = s;
    t ^= t << 11;
    t ^= t >> 8;
    return mSeed[0] = t ^ s ^ (s >> 19);
}

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
        theta = 0.25f * PI * (uOffset.y / uOffset.x);
    }
    else
    {
        r = uOffset.y;
        theta = 0.5f * PI * (1.0f - 0.5f * (uOffset.x / uOffset.y));
    }
    return float2(r * std::cos(theta), r * std::sin(theta));
}

float3
sampleCosineWeightedHemisphere(float& pdf, const float2& u)
{
    float2 d = sampleConcentricDisk(u);
    float z = std::sqrt(std::max(0.0f, 1.0f - d.x * d.x - d.y * d.y));
    float3 dir = normalize(float3(d.x, d.y, z));
    pdf = z / PI;
    return dir;
}
}