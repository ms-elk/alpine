#pragma once

#include "shape.h"

#include <math/vector.h>

#include <memory>
#include <vector>

namespace alpine {
class Material;

class Sphere final : public Shape
{
public:
    struct Data
    {
        std::vector<float4> vertices;
        std::vector<std::shared_ptr<Material>> materials;
    };

    Sphere(const std::shared_ptr<Data>& data)
        : mData(data) {}

    void appendTo(Accelerator* accelerator) const override;

    IntersectionAttributes getIntersectionAttributes(
        const Intersection& isect) const override;

private:
    std::shared_ptr<Data> mData;
};
}