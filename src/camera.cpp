#include "camera.h"

#include "ray.h"
#include "vector.h"

namespace alpine {
constexpr float FILM_DIST = 1.0f;

void
Camera::set(
    const float3& eye,
    const float3& at,
    const float3& up,
    float fovy,
    float aspect)
{
    mEye = eye;

    mFilmHeight = FILM_DIST * std::tan(0.5f * fovy);
    mFilmWidth = mFilmHeight * aspect;

    float3 viewZ = normalize(at - mEye);
    float3 viewX = normalize(cross(up, viewZ));
    float3 viewY = cross(viewZ, viewX);
    mViewMatrix.setColumn(0, viewX);
    mViewMatrix.setColumn(1, viewY);
    mViewMatrix.setColumn(2, viewZ);
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
}