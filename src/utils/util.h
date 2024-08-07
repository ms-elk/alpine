#pragma once

#include "math/vector.h"

#include <algorithm>

namespace alpine {
static constexpr float PI = 3.14159265358979323846f;
static constexpr float ONE_MINUS_EPSILON = 1.0f - std::numeric_limits<float>::epsilon();

inline float cos2Theta(const float3& w) { return w.z * w.z; }
inline float cosTheta(const float3& w) { return w.z; }
inline float sin2Theta(const float3& w) { return std::max(1.0f - cos2Theta(w), 0.0f); }
inline float sinTheta(const float3& w) { return std::sqrt(sin2Theta(w)); }
inline float tan2Theta(const float3& w) { return sin2Theta(w) / cos2Theta(w); }
inline float tanTheta(const float3& w) { return sinTheta(w) / cosTheta(w); }

inline float cosPhi(const float3& w)
{
    float st = sinTheta(w);
    return st == 0.0f ? 1.0f : std::clamp(w.x / st, -1.0f, 1.0f);
}

inline float sinPhi(const float3& w)
{
    float st = sinTheta(w);
    return st == 0.0f ? 0.0f : std::clamp(w.y / st, -1.0f, 1.0f);
}

inline float3 reflect(const float3& v, const float3& n) { return -v + n * 2.0f * dot(v, n); }

inline float3 toLocal(const float3& v, const float3& s, const float3& t, const float3& n)
{
    return float3(dot(s, v), dot(t, v), dot(n, v));
}

inline float3 toWorld(const float3& v, const float3& s, const float3& t, const float3& n)
{
    return s * v.x + t * v.y + n * v.z;
};

inline bool isSameHemisphere(const float3& v0, const float3& v1) { return v0.z * v1.z > 0.0f; }

inline float powerHeuristic(uint32_t na, float aPdf, uint32_t nb, float bPdf)
{
    float a = na * aPdf;
    float b = nb * bPdf;
    return (a * a) / (a * a + b * b);
}

inline float3 schlickFresnel(const float3& f0, const float3& wi, const float3& wh)
{
    float f = pow(1.0f - dot(wi, wh), 5.0f);
    return float3(f) + f0 * (1.0f - f);
}

float2 sampleConcentricDisk(const float2& u);
std::pair<float3 /* dir */, float /* pdf */> sampleCosineWeightedHemisphere(const float2& u);
std::pair<float3 /* dir*/, float /* pdf */> sampleHemisphere(const float2& u);
}
