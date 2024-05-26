#pragma once

#include "math/vector.h"

#include <vector>

namespace alpine {
struct Ray;

namespace kernel {
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
    void* shapePtr;
    uint32_t primId;
    float3 ng;
    float2 barycentric;
    float t;

};
Intersection intersect(const Ray& ray);

bool occluded(const Ray& ray, float far);
}
}
