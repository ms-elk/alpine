#include "accelerators/embree.h"

#include <intersection.h>

namespace alpine {
class Embree::Impl
{};

Embree::Embree()
{
    printf("ERROR: Embree is not available in this build.\n");
}

Embree::~Embree() = default;

uint32_t
Embree::appendMesh(
    const std::vector<float3>& vertices,
    const std::vector<uint3>& prims,
    const void* ptr)
{
    return INVALID_SHAPE_ID;
}

void
Embree::appendSphere(const std::vector<float4>& vertices, const void* ptr)
{}

void*
Embree::getVertexBuffer(uint32_t shapeId)
{
    return nullptr;
}

void
Embree::updateShape(uint32_t shapeId)
{
}

void
Embree::updateScene()
{}

std::optional<Intersection>
Embree::intersect(const Ray& ray) const
{
    return {};
}

bool
Embree::intersectAny(const Ray& ray, float far) const
{
    return false;
}
}
