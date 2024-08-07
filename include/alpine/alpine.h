#pragma once

#include <alpine/camera.h>
#include <alpine/light.h>

#include <stdint.h>
#include <string_view>

namespace alpine {
void initialize(uint32_t width, uint32_t height, uint32_t maxDepth);

enum class FileType {
    Gltf,
    Obj,
};
bool load(std::string_view filename, FileType fileType);

api::Light* addPointLight(const float intensity[3], const float position[3]);

api::Light* addDiskLight(const float emission[3], const float position[3], float radius);

enum class LightSamplerType {
    Uniform,
    Power,
};
void buildLightSampler(LightSamplerType lightSamplerType);

void setBackgroundColor(float r, float g, float b);

api::Camera* getCamera();

// Render
void resetAccumulation();
void render(uint32_t spp);

void resolve(bool denoise);

const void* getFrameBuffer();
void saveImage(std::string_view filename);

// Debug
void addDebugScene();
}
