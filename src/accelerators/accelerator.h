#pragma once

#include "math/vector.h"

#include <vector>

namespace alpine {
struct Ray;

struct Intersection
{
    const void* shapePtr = nullptr;
    uint32_t primId = std::numeric_limits<uint32_t>::max();
    float3 ng;
    float2 barycentric;
    float t = std::numeric_limits<float>::max();

};

class Accelerator
{
public:
    Accelerator() = default;
    virtual ~Accelerator() = default;

    virtual void appendMesh(
        const std::vector<float3>& vertices,
        const std::vector<uint3>& prims,
        const void* ptr) = 0;

    virtual void appendSphere(
        const std::vector<float4>& vertices, const void* ptr) = 0;

    virtual void updateScene() = 0;

    virtual Intersection intersect(const Ray& ray) const = 0;

    virtual bool occluded(const Ray& ray, float far) const = 0;
};
}
