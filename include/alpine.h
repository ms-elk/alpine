#pragma once

namespace alpine {
void initialize(int width, int height, int maxDepth);

bool loadObj(const char* filename);

void setBackgroundColor(float r, float g, float b);

// Camera
void setCameraLookAt(
    const float eye[3],
    const float target[3],
    const float up[3],
    float fovy,
    float aspect);
void orbitCamera(float theta, float phi);
void zoomCamera(float z);
void panCamera(float x, float y);

// Render
void resetAccumulation();
void render(int spp);

const void* getFrameBuffer();
void saveImage(const char* filename);

// debug
void addDebugScene();
}