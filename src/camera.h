#pragma once

#include "matrix.h"
#include "vector.h"

namespace alpine {
struct Ray;

class Camera
{
public:
    Camera() {};

    void setLookAt(
        const float3& eye,
        const float3& target,
        const float3& up,
        float fovy,
        float aspect);

    void orbit(float theta, float phi);
    void zoom(float z);
    void pan(float x, float y);

    Ray generateRay(float x, float y) const;

private:
    void updateViewMatrix();

private:
    float3 mEye;
    float3 mTarget;
    float3 mUp;
    float mFilmWidth = 0.0f;
    float mFilmHeight = 0.0f;
    float3x3 mViewMatrix;
};
}