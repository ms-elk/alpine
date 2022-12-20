#pragma once

#include <memory>

namespace alpine {
class Mesh;

std::shared_ptr<Mesh> createDebugTriangle();
}