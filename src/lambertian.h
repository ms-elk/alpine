#pragma once

#include "material.h"

namespace alpine {
class Lambertian : public Material
{
public:
    Lambertian (const Vector3f& albedo) : mAlbedo(albedo) {}
    virtual ~Lambertian() {}

    virtual Vector3f evaluate(const Vector3f& wo,const Vector3f& wi) const override;

    virtual Vector3f sample(
        const Vector3f& wo,
        const Vector3f& ng,
        const Vector2f& u,
        Vector3f& wi,
        float& pdf) const override;

private:
    Vector3f mAlbedo;
};
}