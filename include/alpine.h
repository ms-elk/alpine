#pragma once

#include <i_camera.h>
#include <i_light.h>

#include <stdint.h>
#include <string_view>

namespace alpine {
void initialize(uint32_t width, uint32_t height, uint32_t maxDepth);

enum class FileType {
    GLTF,
    OBJ,
};
bool load(std::string_view filename, FileType fileType);

ILight* addPointLight(const float intensity[3], const float position[3]);

ILight* addDiskLight(const float emission[3], const float position[3], float radius);

void setBackgroundColor(float r, float g, float b);

ICamera* getCamera();

// Render
void resetAccumulation();
void render(uint32_t spp);

void resolve(bool denoise);

const void* getFrameBuffer();
void saveImage(std::string_view filename);

// Debug
void addDebugScene();
}
