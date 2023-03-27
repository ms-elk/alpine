#pragma once

#include "matrix.h"
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

    void rotate(float theta, float phi);

    Ray generateRay(float x, float y) const;

private:
    void updateViewMatrix();

private:
    float3 mEye;
    float3 mAt;
    float3 mUp;
    float mFilmWidth = 0.0f;
    float mFilmHeight = 0.0f;
    float3x3 mViewMatrix;
};
}