#include "camera.h"

#include "ray.h"
#include "vector.h"

namespace alpine {
constexpr float FILM_DIST = 1.0f;

void
Camera::set(
    const Vector3f& eye,
    const Vector3f& at,
    const Vector3f& up,
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

    const auto toWorld = [&](const Vector3f& v)
    {
        return mBase[0] * v.x + mBase[1] * v.y + mBase[2] * v.z;
    };
    Vector3f dir = normalize(toWorld(Vector3f(filmX, filmY, FILM_DIST)));

    Ray ray;
    ray.org = mEye;
    ray.dir = dir;

    return ray;
}
}