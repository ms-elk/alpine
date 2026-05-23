#pragma once

#include <string_view>

namespace alpine {
class EnvironmentMap;
struct Scene;

bool loadGltf(Scene* scene, std::string_view filename);
bool loadObj(Scene* scene, std::string_view filename);
bool loadHdr(std::unique_ptr<EnvironmentMap>& environmentMap, std::string_view filename);
}
