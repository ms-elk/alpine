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
}
