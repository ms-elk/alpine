#pragma once

#include "shape.h"
#include "vector.h"

#include <memory>
#include <vector>

namespace alpine {
class Material;

class Mesh : public Shape
{
public:
    struct Data
    {
        std::vector<float3> vertices;
        std::vector<uint3> prims;
        std::vector<std::shared_ptr<Material>> materials;
    };

    Mesh(const std::shared_ptr<Data>& data);
    virtual ~Mesh() override {}

    virtual IntersectionAttributes getIntersectionAttributes(
        const Ray& ray, const kernel::Intersection& isect) const override;

private:
    std::shared_ptr<Data> mData;
};
}