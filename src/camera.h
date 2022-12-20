#pragma once

#include "vector.h"

namespace alpine {
struct Ray;

class Camera
{
public:
    Camera() {};

    void set(
        const Vector3f& eye,
        const Vector3f& at,
        const Vector3f& up,
        float fovy,
        float aspect);

    Ray generateRay(float x, float y) const;

private:
    Vector3f mEye;
    float mFilmWidth = 0.0f;
    float mFilmHeight = 0.0f;
    Vector3f mBase[3];
};
}