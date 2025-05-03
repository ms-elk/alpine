#include "bvh4.h"

namespace alpine {
class Bvh4::Impl
{};

Bvh4::Bvh4()
{
    printf("ERROR: Bvh4 is not available in this build.\n");
}

Bvh4::~Bvh4() = default;

void
Bvh4::appendMesh(
    const std::vector<float3>& vertices,
    const std::vector<uint3>& prims,
    const void* ptr)
{}

void
Bvh4::appendSphere(const std::vector<float4>& vertices, const void* ptr)
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
