#pragma once

#include <alpine/camera.h>
#include <alpine/light.h>

#include <stdint.h>
#include <string_view>

namespace alpine {
enum class AcceleratorType {
    Bvh,
    WideBvh,
    Embree,
};

bool initialize(
    uint32_t memoryArenaSize,
    uint32_t width,
    uint32_t height,
    uint32_t maxDepth,
    AcceleratorType acceleratorType);

enum class FileType {
    Gltf,
    Obj,
};
bool load(std::string_view filename, FileType fileType);

void updateScene(float time);

bool isDynamicScene();

api::Light* addPointLight(float power, const float color[3], const float position[3]);

api::Light* addDiskLight(
    float power,
    const float color[3],
    const float position[3],
    const float target[3],
    float radius);

enum class LightSamplerType {
    Uniform,
    Power,
    Bvh,
};

// call this function after all of the lights have benn added to the scene
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
