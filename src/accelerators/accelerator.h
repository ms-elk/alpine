#pragma once

#include "math/vector.h"

#include <vector>

namespace alpine {
struct Ray;

namespace accelerator {
bool initialize();

void finalize();

bool createMesh(
    const std::vector<float3>& vertices,
    const std::vector<uint3>& prims,
    void* ptr);

bool createSphere(const std::vector<float4>& vertices, void* ptr);

void updateScene();

struct Intersection
{
    void* shapePtr = nullptr;
    uint32_t primId = std::numeric_limits<uint32_t>::max();
    float3 ng;
    float2 barycentric;
    float t = std::numeric_limits<float>::max();

};
Intersection intersect(const Ray& ray);

bool occluded(const Ray& ray, float far);
}
}
