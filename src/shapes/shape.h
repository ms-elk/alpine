#pragma once

#include <math/vector.h>

namespace alpine {
class Material;
class Accelerator;
struct Intersection;

struct IntersectionAttributes
{
    float3 ns;
    float3 ss;
    float3 ts;
    float2 uv;
    const Material* material = nullptr;
};

class Shape
{
public:
    Shape() = default;
    virtual ~Shape() = default;

    virtual void appendTo(Accelerator* accelerator) = 0;

    virtual void update(Accelerator* accelerator, float weight) = 0;

    virtual IntersectionAttributes getIntersectionAttributes(
        const Intersection& isect) const = 0;
};
}
