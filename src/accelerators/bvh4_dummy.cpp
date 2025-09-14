#include "bvh4.h"

#include <intersection.h>

namespace alpine {
class Bvh4::Impl
{};

Bvh4::Bvh4(std::span<std::byte> memoryArenaBuffer)
{
    printf("ERROR: Bvh4 is not available in this build.\n");
}

Bvh4::~Bvh4() = default;

uint32_t
Bvh4::appendMesh(
    const std::vector<float3>& vertices,
    const std::vector<uint3>& prims,
    const void* ptr)
{
    return INVALID_SHAPE_ID;
}

void
Bvh4::appendSphere(const std::vector<float4>& vertices, const void* ptr)
{}

void*
Bvh4::getVertexBuffer(uint32_t shapeId)
{
    return nullptr;
}

void
Bvh4::updateShape(uint32_t shapeId)
{}

void
Bvh4::updateScene()
{}

std::optional<Intersection>
Bvh4::intersect(const Ray& ray) const
{
    return {};
}

bool
Bvh4::intersectAny(const Ray& ray, float tFar) const
{
    return false;
}
}
