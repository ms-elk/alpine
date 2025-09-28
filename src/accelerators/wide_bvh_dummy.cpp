#include "wide_bvh.h"

#include <intersection.h>

namespace alpine {
class WideBvh::Impl
{};

WideBvh::WideBvh(std::span<std::byte> memoryArenaBuffer)
{
    printf("ERROR: WideBvh is not available in this build.\n");
}

WideBvh::~WideBvh() = default;

uint32_t
WideBvh::appendMesh(
    const std::vector<float3>& vertices,
    const std::vector<uint3>& prims,
    const void* ptr)
{
    return INVALID_SHAPE_ID;
}

void
WideBvh::appendSphere(const std::vector<float4>& vertices, const void* ptr)
{}

void*
WideBvh::getVertexBuffer(uint32_t shapeId)
{
    return nullptr;
}

void
WideBvh::updateShape(uint32_t shapeId)
{}

void
WideBvh::updateScene()
{}

std::optional<Intersection>
WideBvh::intersect(const Ray& ray) const
{
    return {};
}

bool
WideBvh::intersectAny(const Ray& ray, float tFar) const
{
    return false;
}
}
