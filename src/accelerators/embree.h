#pragma once

#include "accelerator.h"

#include <memory>
#include <vector>

namespace alpine {
struct Ray;

class Embree : public Accelerator
{
public:
    Embree();

    virtual void appendMesh(
        const std::vector<float3>& vertices,
        const std::vector<uint3>& prims,
        const void* ptr) override;

    virtual void appendSphere(
        const std::vector<float4>& vertices, const void* ptr) override;

    virtual void updateScene() override;

    virtual std::optional<Intersection> intersect(const Ray& ray) const override;

    virtual bool intersectAny(const Ray& ray, float tFar) const override;

private:
    class Impl;
    std::unique_ptr<Impl> mPimpl;
};
}
