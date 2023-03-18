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

    auto view = normalize(at - eye);
    mBase[2] = view;
    mBase[0] = normalize(cross(up, mBase[2]));
    mBase[1] = cross(mBase[2], mBase[0]);
}

Ray
Camera::generateRay(float x, float y) const
{
    float filmX = (x - 0.5f) * mFilmWidth;
    float filmY = (0.5f - y) * mFilmHeight;

    const auto toWorld = [&](const float3& v)
    {
        return mBase[0] * v.x + mBase[1] * v.y + mBase[2] * v.z;
    };
    float3 dir = normalize(toWorld(float3(filmX, filmY, FILM_DIST)));

    Ray ray;
    ray.org = mEye;
    ray.dir = dir;

    return ray;
}
}