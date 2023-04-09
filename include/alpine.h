#pragma once

#include <stdint.h>

namespace alpine {
void initialize(uint32_t width, uint32_t height, uint32_t maxDepth);

bool loadObj(const char* filename);

void setBackgroundColor(float r, float g, float b);

class ICamera
{
public:
    virtual void setLookAt(
        const float eye[3],
        const float target[3],
        const float up[3],
        float fovy,
        float aspect) = 0;
    virtual void orbit(float theta, float phi) = 0;
    virtual void zoom(float z) = 0;
    virtual void pan(float x, float y) = 0;
};
ICamera* getCamera();

// Render
void resetAccumulation();
void render(uint32_t spp);

const void* getFrameBuffer();
void saveImage(const char* filename);

// debug
void addDebugScene();
}