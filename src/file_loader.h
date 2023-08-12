#pragma once

namespace alpine {
struct Scene;
bool loadGltf(Scene* scene, const char* filename);
bool loadObj(Scene* scene, const char* filename);
}
