﻿#pragma once

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

class Bvh final : public Accelerator
{
public:
    Bvh();
    ~Bvh() override;

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
    std::unique_ptr<BuildNode> buildBvh(
        const std::vector<BuildPrimitive>& bvhPrimitives,
        std::atomic<uint32_t>& offset,
        std::atomic<uint32_t>& nodeCount);

    std::optional<bvh_util::Split> findSplit(const std::vector<BuildPrimitive>& bvhPrimitives);

    uint32_t flatten(const BuildNode* node, uint32_t& offset);

    std::optional<Intersection> traverse(const Ray& ray, float tFar, bool any) const;

private:
    std::vector<Primitive> mPrimitives;
    std::vector<Primitive> mOrderedPrimitives;
    std::vector<LinearNode> mLinearNodes;
};
}
