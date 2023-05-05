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

float2 sampleConcentricDisk(const float2& u);
float3 sampleCosineWeightedHemisphere(float& pdf, const float2& u);
}