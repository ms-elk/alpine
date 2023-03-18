#pragma once

#include "vector.h"

namespace alpine {
struct Ray;

class Camera
{
public:
    Camera() {};

    void set(
        const float3& eye,
        const float3& at,
        const float3& up,
        float fovy,
        float aspect);

    Ray generateRay(float x, float y) const;

private:
    float3 mEye;
    float mFilmWidth = 0.0f;
    float mFilmHeight = 0.0f;
    float3 mBase[3];
};
}