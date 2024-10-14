#include "camera.h"

#include "math/vector.h"
#include "ray.h"
#include "utils/util.h"

#include <algorithm>
#include <numbers>

namespace alpine {
constexpr float FILM_DIST = 1.0f;

void
Camera::setLookAt(
    const float eye[3],
    const float target[3],
    const float up[3],
    float fovy,
    float aspect)
{
    mEye = float3(eye);
    mTarget = float3(target);
    mUp = float3(up);

    mFilmHeight = FILM_DIST * std::tan(0.5f * fovy);
    mFilmWidth = mFilmHeight * aspect;

    updateViewMatrix();
}

void
Camera::orbit(float theta, float phi)
{
    float3 toEye = mEye - mTarget;
    float toEyeDist = length(toEye);
    if (toEyeDist == 0.0f)
    {
        return;
    }

    toEye /= toEyeDist;

    theta = std::atan2(toEye.x, toEye.z) - theta;
    phi = std::clamp(std::acos(toEye.y) - phi, 0.0f, std::numbers::pi_v<float>);

    float sinTheta = std::sin(theta);
    float cosTheta = std::cos(theta);
    float sinPhi = std::sin(phi);
    float cosPhi = std::cos(phi);
    toEye.x = sinTheta * sinPhi * toEyeDist;
    toEye.y = cosPhi * toEyeDist;
    toEye.z = cosTheta * sinPhi * toEyeDist;

    mEye = toEye + mTarget;

    updateViewMatrix();
}

void
Camera::zoom(float z)
{
    mEye -= mViewMatrix.getColumn(2) * z;
}

void
Camera::pan(float x, float y)
{
    float3 translation = mViewMatrix.getColumn(0) * x - mViewMatrix.getColumn(1) * y;
    mEye -= translation;
    mTarget -= translation;
}

Ray
Camera::generateRay(float x, float y) const
{
    float filmX = (x - 0.5f) * mFilmWidth;
    float filmY = (0.5f - y) * mFilmHeight;
    float3 dir = normalize(mul(mViewMatrix, float3(filmX, filmY, FILM_DIST)));

    Ray ray;
    ray.org = mEye;
    ray.dir = dir;

    return ray;
}

void
Camera::updateViewMatrix()
{
    float3 viewZ = normalize(mTarget - mEye);
    float3 viewX = normalize(cross(viewZ, mUp));
    float3 viewY = cross(viewX, viewZ);
    mViewMatrix.setColumn(0, viewX);
    mViewMatrix.setColumn(1, viewY);
    mViewMatrix.setColumn(2, viewZ);
}
}