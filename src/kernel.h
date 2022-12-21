#pragma once

#include "vector.h"

#include <vector>

namespace alpine {
struct Ray;

namespace kernel {
bool initialize();

void finalize();

bool createMesh(
    const std::vector<Vector3f>& vertices,
    const std::vector<Vector3ui>& prims,
    void* ptr);

bool createSphere(const std::vector<Vector4f>& vertices, void* ptr);

void updateScene();

struct Intersection
{
    void* shapePtr;
    unsigned int primId;
    Vector3f ng;
    float u;
    float v;
    float t;

};
Intersection intersect(const Ray& ray);
}
}