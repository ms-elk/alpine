#include "accelerators/accelerator.h"

#include "ray.h"

namespace alpine::accelerator {
bool
initialize()
{
    return true;
}

void
finalize()
{
}

bool
createMesh(
    const std::vector<float3>& vertices,
    const std::vector<uint3>& prims,
    void* ptr)
{
    return true;
}

bool
createSphere(const std::vector<float4>& vertices, void* ptr)
{
    return true;
}

void
updateScene()
{
}

Intersection
intersect(const Ray& ray)
{
    Intersection isect;
    isect.shapePtr = nullptr;
    return isect;
}

bool
occluded(const Ray& ray, float far)
{
    return false;
}
}
