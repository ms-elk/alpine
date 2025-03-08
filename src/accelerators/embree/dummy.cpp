#include "accelerators/embree.h"

namespace alpine {
class Embree::Impl
{};

Embree::Embree()
{
    printf("ERROR: Embree is not available in this build.\n");
}

void
Embree::appendMesh(
    const std::vector<float3>& vertices,
    const std::vector<uint3>& prims,
    const void* ptr)
{}

void
Embree::appendSphere(const std::vector<float4>& vertices, const void* ptr)
{}

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
