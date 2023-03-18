#pragma once

#include "vector.h"

namespace alpine {
float2 get2D();
float2 sampleConcentricDisk(const float2& u);
float3 sampleCosineWeightedHemisphere(float& pdf, const float2& u);
}