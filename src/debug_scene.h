#pragma once

#include <memory>

namespace alpine {
class Mesh;
class Sphere;

std::shared_ptr<Mesh> createDebugTriangle();

std::shared_ptr<Sphere> createDebugSphere();
}