#pragma once

#include "accelerator.h"

#include <memory>
#include <vector>

namespace alpine {
struct Ray;

class Embree final : public Accelerator
{
public:
    Embree();
    ~Embree() override;

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
    class Impl;
    std::unique_ptr<Impl> mPimpl;
};
}
