#pragma once

#include <memory>
#include <vector>

namespace alpine {
class Mesh;
struct Scene;

std::shared_ptr<Mesh> createMesh(const char* filename);
bool createMeshes(Scene& scene, const char* filename);
}