#pragma once

#include "shape.h"
#include "math/vector.h"

#include <memory>
#include <vector>

namespace alpine {
class Material;
struct Intersection;

class Mesh final : public Shape
{
public:
    struct Data
    {
        std::vector<float3> vertices;
        std::vector<float3> normals;
        std::vector<float3> tangents;
        std::vector<float3> bitangents;
        std::vector<float2> uvs;
        std::vector<uint3> prims;
        std::vector<uint3> normalPrims;
        std::vector<uint3> uvPrims;
        std::vector<std::shared_ptr<Material>> materials;
    };

    Mesh(Data&& data)
        : mData(std::move(data)) {};

    void appendTo(Accelerator* accelerator) const override;

    IntersectionAttributes getIntersectionAttributes(
        const Intersection& isect) const override;

private:
    Data mData;
};
}