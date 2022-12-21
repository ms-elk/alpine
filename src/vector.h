#pragma once

#include <cmath>

namespace alpine {
template <typename T>
class Vector2
{
public:
    Vector2() { x = y = 0; }
    Vector2(T x, T y) : x(x), y(y) {}
    Vector2(T v) : x(v), y(v) {}
    Vector2(const Vector2<T>& v) : x(v.x), y(v.y) {}
    Vector2(const float v[2]) : x(v[0]), y(v[1]) {}

    Vector2<T>& operator=(const Vector2<T>& v)
    {
        x = v.x;
        y = v.y;

        return *this;
    }

    Vector2<T> operator+(const Vector2<T>& v) const
    {
        return Vector2(x + v.x, y + v.y);
    }

    Vector2<T> operator-(const Vector2<T>& v) const
    {
        return Vector2(x - v.x, y - v.y);
    }

    Vector2<T> operator*(const T f) const
    {
        return Vector2(x * f, y * f);
    }

    Vector2<T> operator/(const T f) const
    {
        return Vector2(x / f, y / f);
    }

public:
    T x, y;
};

using Vector2f = Vector2<float>;

template <typename T>
class Vector3
{
public:
    Vector3() { x = y = z = 0; }
    Vector3(T x, T y, T z) : x(x), y(y), z(z) {}
    Vector3(T v) : x(v), y(v), z(v) {}
    Vector3(const Vector3<T>& v) : x(v.x), y(v.y), z(v.z) {}
    Vector3(const float v[3]) : x(v[0]), y(v[1]), z(v[2]) {}

    Vector3<T>& operator=(const Vector3<T>& v)
    {
        x = v.x;
        y = v.y;
        z = v.z;

        return *this;
    }

    Vector3<T>& operator+=(const Vector3<T>& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;

        return *this;
    }

    Vector3<T>& operator-=(const Vector3<T>& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;

        return *this;
    }

    Vector3<T>& operator*=(const T f)
    {
        x *= f;
        y *= f;
        z *= f;

        return *this;
    }

    Vector3<T>& operator/=(const T f)
    {
        x /= f;
        y /= f;
        z /= f;

        return *this;
    }

    Vector3<T> operator+(const Vector3<T>& v) const
    {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }

    Vector3<T> operator-(const Vector3<T>& v) const
    {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }

    Vector3<T> operator*(const Vector3<T>& v) const
    {
        return Vector3(x * v.x, y * v.y, z * v.z);
    }

    Vector3<T> operator*(const T f) const
    {
        return Vector3(x * f, y * f, z * f);
    }

    Vector3<T> operator/(const T f) const
    {
        return Vector3(x / f, y / f, z / f);
    }

public:
    T x, y, z;
};

template <typename T>
T dot(const Vector3<T>& v0, const Vector3<T>& v1)
{
    return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z;
}

template <typename T>
T length(const Vector3<T>& v)
{
    return std::sqrt(dot(v, v));
}

template <typename T>
Vector3<T> cross(const Vector3<T>& v0, const Vector3<T>& v1)
{
    float x = v0.y * v1.z - v0.z * v1.y;
    float y = v0.z * v1.x - v0.x * v1.z;
    float z = v0.x * v1.y - v0.y * v1.x;

    return Vector3<T>(x, y, z);
}

template <typename T>
Vector3<T> normalize(const Vector3<T>& v)
{
    return v / length(v);
}

template <typename T>
Vector3<T> transformBasis(const Vector3<T>& v, const Vector3<T>& basis)
{
    Vector3<T> tv;

    Vector3<T> up(0, 0, 1);
    Vector3<T> tan = cross(basis, up);
    if (length(tan) > 0.001f)
    {
        tan = normalize(tan);
        Vector3<T> bi = cross(tan, basis);
        tv = tan * v.x + bi * v.y + basis * v.z;
    }
    else
    {
        tv = basis.z >= 0.0f ? v : v * -1.0f;
    }

    return tv;
}

using Vector3f = Vector3<float>;
using Vector3ui = Vector3<unsigned int>;

template <typename T>
class Vector4
{
public:
    Vector4() { x = y = z = w = 0; }
    Vector4(T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {}
    Vector4(T v) : x(v), y(v), z(v), w(v) {}
    Vector4(const Vector4<T>& v) : x(v.x), y(v.y), z(v.z), w(v.w) {}
    Vector4(const float v[4]) : x(v[0]), y(v[1]), z(v[2]), w(v[3]) {}

    Vector4<T>& operator=(const Vector4<T>& v)
    {
        x = v.x;
        y = v.y;
        z = v.z;
        w = v.w;

        return *this;
    }

    Vector4<T> operator+(const Vector4<T>& v) const
    {
        return Vector4(x + v.x, y + v.y, z + v.z, w + v.w);
    }

    Vector4<T> operator-(const Vector4<T>& v) const
    {
        return Vector4(x - v.x, y - v.y, z - v.z, w - v.w);
    }

    Vector4<T> operator*(const T f) const
    {
        return Vector4(x * f, y * f, z * f, w * f);
    }

    Vector4<T> operator/(const T f) const
    {
        return Vector4(x / f, y / f, z / f, w / f);
    }

public:
    T x, y, z, w;
};

using Vector4f = Vector4<float>;
}