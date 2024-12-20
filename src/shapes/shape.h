﻿#pragma once

#include "math/vector.h"

namespace alpine {
namespace accelerator {
struct Intersection;
}
class Material;

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

    virtual IntersectionAttributes getIntersectionAttributes(
        const accelerator::Intersection& isect) const = 0;
};
}
