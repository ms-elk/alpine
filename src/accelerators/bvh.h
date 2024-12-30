#pragma once

#include "accelerator.h"

#include <memory>
#include <vector>

namespace alpine {
struct Ray;
struct Primitive;
struct Node;

class Bvh : public Accelerator
{
public:
    Bvh();

    virtual void appendMesh(
        const std::vector<float3>& vertices,
        const std::vector<uint3>& prims,
        const void* ptr) override;

    virtual void appendSphere(
        const std::vector<float4>& vertices, const void* ptr) override;

    virtual void updateScene() override;

    virtual Intersection intersect(const Ray& ray) const override;

    virtual bool occluded(const Ray& ray, float far) const override;

private:
    std::vector<Primitive> mPrimitives;
    std::unique_ptr<Node> mBvh = nullptr;
};
}
