﻿#pragma once

#include "accelerator.h"

#include <memory>

namespace alpine {
struct Ray;

class Bvh final : public Accelerator
{
public:
    Bvh();
    ~Bvh() override;

    uint32_t appendMesh(
        const std::vector<float3>& vertices,
        const std::vector<uint3>& prims,
        const void* ptr) override;

    void appendSphere(
        const std::vector<float4>& vertices, const void* ptr) override;

    void* getVertexBuffer(uint32_t shapeId) override { return nullptr; }

    void updateShape(uint32_t shapeId) override {}

    void updateScene() override;

    std::optional<Intersection> intersect(const Ray& ray) const override;

    bool intersectAny(const Ray& ray, float tFar) const override;

private:
    class Impl;
    std::unique_ptr<Impl> mPimpl;
};
}
