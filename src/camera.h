#pragma once

#include <math/matrix.h>
#include <math/vector.h>

#include <alpine/camera.h>

namespace alpine {
struct Ray;

class Camera : public api::Camera
{
public:
    Camera() {};

    void setLookAt(
        const float eye[3],
        const float target[3],
        const float up[3],
        float fovy,
        float aspect) override;

    void orbit(float theta, float phi) override;
    void zoom(float z) override;
    void pan(float x, float y) override;

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