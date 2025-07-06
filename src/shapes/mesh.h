#pragma once

#include "shape.h"
#include <math/vector.h>

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

        struct Target
        {
            std::vector<float3> vertices;
            std::vector<float3> normals;
            std::vector<float3> tangents;
            std::vector<float3> bitangents;
        };
        std::vector<Target> targets;
    };

    Mesh(Data&& data);

    void appendTo(Accelerator* accelerator) override;

    void update(Accelerator* accelerator, float weight) override;

    IntersectionAttributes getIntersectionAttributes(
        const Intersection& isect) const override;

private:
    Data mData;
    uint32_t mShapeId;
    float3* mVertexBuffer = nullptr;
    float mWeight = 0.0f;
};
}