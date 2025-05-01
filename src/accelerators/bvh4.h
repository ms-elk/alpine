#pragma once

#include "accelerator.h"

#include <atomic>
#include <memory>
#include <optional>
#include <vector>

namespace alpine {
struct Ray;
struct Primitive4;
struct BoundingBox;
struct BuildPrimitive4;
struct BuildNode4;
struct LinearNode4;

class Bvh4 final : public Accelerator
{
public:
    Bvh4();
    ~Bvh4() override;

    void appendMesh(
        const std::vector<float3>& vertices,
        const std::vector<uint3>& prims,
        const void* ptr) override;

    void appendSphere(
        const std::vector<float4>& vertices, const void* ptr) override;

    void updateScene() override;

    std::optional<Intersection> intersect(const Ray& ray) const override;

    bool intersectAny(const Ray& ray, float tFar) const override;

private:
    std::unique_ptr<BuildNode4> buildBvh(
        const std::vector<BuildPrimitive4>& buildPrimitives,
        std::atomic<uint32_t>& offset,
        std::atomic<uint32_t>& nodeCount);

    std::unique_ptr<BuildNode4> createLeaf(
        const std::vector<BuildPrimitive4>& buildPrimitives,
        const BoundingBox& bbox,
        std::atomic<uint32_t>& offset);

    uint32_t flatten(const BuildNode4* node, uint32_t& offset);

    std::optional<Intersection> traverse(const Ray& ray, float tFar, bool any) const;

private:
    std::vector<Primitive4> mPrimitives;
    std::vector<Primitive4> mOrderedPrimitives;
    std::vector<LinearNode4> mLinearNodes;
};
}
