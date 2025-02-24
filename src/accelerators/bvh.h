#pragma once

#include "accelerator.h"

#include <atomic>
#include <memory>
#include <optional>
#include <vector>

namespace alpine {
struct Ray;
struct Primitive;
struct BuildPrimitive;
struct BuildNode;
struct LinearNode;

namespace bvh_util {
struct Split;
}

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

    virtual std::optional<Intersection> intersect(const Ray& ray) const override;

    virtual bool occluded(const Ray& ray, float tFar) const override;

private:
    std::unique_ptr<BuildNode> buildBvh(
        const std::vector<BuildPrimitive>& bvhPrimitives,
        std::atomic<uint32_t>& offset,
        std::atomic<uint32_t>& nodeCount);

    std::optional<bvh_util::Split> findSplit(
        const std::vector<BuildPrimitive>& bvhPrimitives, const float3& diagonal);

    uint32_t flatten(const BuildNode* node, uint32_t& offset);

    std::optional<Intersection> traverse(const Ray& ray, float tFar, bool any) const;

private:
    std::vector<Primitive> mPrimitives;
    std::vector<Primitive> mOrderedPrimitives;
    std::vector<LinearNode> mLinearNodes;
};
}
