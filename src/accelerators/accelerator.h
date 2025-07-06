#pragma once

#include <math/vector.h>

#include <algorithm>
#include <optional>
#include <vector>

namespace alpine {
struct Intersection;
struct Ray;

class Accelerator
{
public:
    Accelerator() = default;
    virtual ~Accelerator() = default;

    virtual uint32_t appendMesh(
        const std::vector<float3>& vertices,
        const std::vector<uint3>& prims,
        const void* ptr) = 0;

    virtual void appendSphere(
        const std::vector<float4>& vertices, const void* ptr) = 0;

    virtual void* getVertexBuffer(uint32_t shapeId) = 0;

    virtual void updateShape(uint32_t shapeId) = 0;

    virtual void updateScene() = 0;

    virtual std::optional<Intersection> intersect(const Ray& ray) const = 0;

    virtual bool intersectAny(const Ray& ray, float tFar) const = 0;

public:
    static constexpr uint32_t INVALID_SHAPE_ID = std::numeric_limits<uint32_t>::max();
};
}
