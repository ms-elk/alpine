#pragma once

#include <string_view>

namespace alpine {
struct Scene;
bool loadGltf(Scene* scene, std::string_view filename);
bool loadObj(Scene* scene, std::string_view filename);
}
