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
        std::vector<float4> tangents;
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
        };
        std::vector<Target> targets;
    };

    Mesh(Data&& data, uint32_t weightCount);

    void appendTo(Accelerator* accelerator) override;

    void update(
        Accelerator* accelerator,
        const std::vector<float>& weights0,
        const std::vector<float>& weights1,
        float t) override;

    IntersectionAttributes getIntersectionAttributes(
        const Intersection& isect) const override;

private:
    const Data mData;
    uint32_t mShapeId;
    float3* mVertexBuffer = nullptr;
    std::vector<float> mWeights;
};
}
