#pragma once

#include "vector.h"

namespace alpine {
namespace kernel {
struct Intersection;
}

class Material
{
public:
    Material() {}
    virtual ~Material() {};

    // TODO: implement BSDF class
    virtual Vector3f evaluate(const Vector3f& wo, const Vector3f& wi) const = 0;

    virtual Vector3f sample(
        const Vector3f& wo,
        const Vector3f& ng,
        const Vector2f& u,
        Vector3f& wi,
        float& pdf) const = 0;
};
}