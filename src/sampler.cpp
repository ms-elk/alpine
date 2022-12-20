#include "sampler.h"

#include "util.h"
#include <algorithm>

namespace {
float getRandom()
{
    return rand() / (RAND_MAX + 1.0f);
}
}

namespace alpine {
Vector2f get2D()
{
    // TODO: implement a better sampler
    return Vector2f(getRandom(), getRandom());
}

Vector2f sampleConcentricDisk(const Vector2f& u)
{
    // Map uniform random numbers to $[-1,1]^2$
    Vector2 uOffset = { 2.f * u.x - 1, 2.f * u.y - 1 };

    // Handle degeneracy at the origin
    if (uOffset.x == 0 && uOffset.y == 0) return Vector2f{ 0, 0 };

    // Apply concentric mapping to point
    float theta, r;
    if (std::abs(uOffset.x) > std::abs(uOffset.y)) {
        r = uOffset.x;
        theta = 0.25f * PI * (uOffset.y / uOffset.x);
    }
    else {
        r = uOffset.y;
        theta = 0.5f * PI * (1.0f - 0.5f * (uOffset.x / uOffset.y));
    }
    return Vector2{ r * std::cos(theta), r * std::sin(theta) };
}

Vector3f sampleCosineWeightedHemisphere(float& pdf, const Vector2f& u)
{
    Vector2f d = sampleConcentricDisk(u);
    float z = std::sqrt(std::max(0.0f, 1.0f - d.x * d.x - d.y * d.y));
    Vector3f dir = normalize(Vector3f(d.x, d.y, z));
    pdf = z / PI;
    return dir;
}
}