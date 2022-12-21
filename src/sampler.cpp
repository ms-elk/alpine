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
    // map uniform random numbers to $[-1,1]^2$
    Vector2f uOffset(2.0f * u.x - 1.0f, 2.0f * u.y - 1.0f);

    // handle degeneracy at the origin
    if (uOffset.x == 0 && uOffset.y == 0)
    {
        return Vector2f(0.0f);
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
    return Vector2f(r * std::cos(theta), r * std::sin(theta));
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