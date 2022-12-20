#pragma once

namespace alpine {
void initialize(int width, int height, int maxDepth);

bool loadObj(const char* filename);

void setCamera(
    const float eye[3],
    const float at[3],
    const float up[3],
    float fovy,
    float aspect);

void render(int spp);

void saveImage(const char* filename);

// debug
void addDebugScene();
}