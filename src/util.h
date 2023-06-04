#pragma once

#include "vector.h"

namespace alpine {
static constexpr float PI = 3.14159265358979323846f;

inline float cos2Theta(const float3& w) { return w.z * w.z; }
inline float cosTheta(const float3& w) { return w.z; }
inline float sin2Theta(const float3& w) { return 1.0f - cos2Theta(w); }
inline float sinTheta(const float3& w) { return std::sqrt(sin2Theta(w)); }
inline float tan2Theta(const float3& w) { return sin2Theta(w) / cos2Theta(w); }
inline float tanTheta(const float3& w) { return sinTheta(w) / cosTheta(w); }

inline float cosPhi(const float3& w)
{
    float st = sinTheta(w);
    return st == 0.0f ? 1.0f : w.x / st;
}

inline float sinPhi(const float3& w)
{
    float st = sinTheta(w);
    return st == 0.0f ? 0.0f : w.y / st;
}

inline float3 reflect(const float3& v, const float3& n)
{
    return v - n * 2.0f * dot(v, n);
}

float2 sampleConcentricDisk(const float2& u);
std::pair<float3 /* dir */, float /* pdf */> sampleCosineWeightedHemisphere(const float2& u);
}
