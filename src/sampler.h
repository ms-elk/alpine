#pragma once

#include "vector.h"

namespace alpine {
class Sampler
{
public:
    Sampler() : mSeed{ 0 } {};

    void reset(uint32_t seed);

    float get1D();
    float2 get2D();

private:
    uint32_t next();

private:
    uint32_t mSeed[4];
};
}
