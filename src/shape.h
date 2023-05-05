#pragma once

#include "vector.h"

namespace alpine {
namespace kernel{
struct Intersection;
}
class Material;

struct IntersectionAttributes
{
    float3 ns; // TODO
    float2 uv;
    Material* material = nullptr;
};

class Shape
{
public:
    Shape(){}
    virtual ~Shape() {}

    virtual IntersectionAttributes getIntersectionAttributes(
        const kernel::Intersection& isect) const = 0;
};
}