#pragma once

#include <cmath>
#include <tuple>

namespace alpine {
template <typename T>
class Vector2
{
public:
    Vector2() { x = y = 0; }
    Vector2(T x, T y) : v{ x, y } {}
    Vector2(T v) : v{ v, v } {}
    Vector2(const Vector2<T>& v) : v{ v.x, v.y } {}
    Vector2(const T v[2]) : v{ v[0], v[1] } {}

    T& operator[](uint32_t i)
    {
        return v[i];
    }

    const T& operator[](uint32_t i) const
    {
        return v[i];
    }

    Vector2<T>& operator=(const Vector2<T>& rhs)
    {
        x = rhs.x;
        y = rhs.y;

        return *this;
    }

    Vector2<T> operator+(const Vector2<T>& rhs) const
    {
        return { x + rhs.x, y + rhs.y };
    }

    Vector2<T> operator-(const Vector2<T>& rhs) const
    {
        return { x - rhs.x, y - rhs.y };
    }

    Vector2<T> operator*(const T rhs) const
    {
        return { x * rhs, y * rhs };
    }

    Vector2<T> operator/(const T rhs) const
    {
        return { x / rhs, y / rhs };
    }

public:
    union
    {
        T v[2];
        struct
        {
            T x, y;
        };
    };
};

template <typename T>
T dot(const Vector2<T>& v0, const Vector2<T>& v1)
{
    return v0.x * v1.x + v0.y * v1.y;
}

using float2 = Vector2<float>;

template <typename T>
class Vector3
{
public:
    Vector3() { x = y = z = 0; }
    Vector3(T x, T y, T z) : v{ x, y, z } {}
    Vector3(T v) : v{ v, v, v } {}
    Vector3(const Vector3<T>& v) : v{ v.x, v.y, v.z } {}
    Vector3(const T v[3]) : v{ v[0], v[1], v[2] } {}

    T& operator[](uint32_t i)
    {
        return v[i];
    }

    const T& operator[](uint32_t i) const
    {
        return v[i];
    }

    Vector3<T>& operator=(const Vector3<T>& rhs)
    {
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;

        return *this;
    }

    Vector3<T>& operator+=(const Vector3<T>& rhs)
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;

        return *this;
    }

    Vector3<T>& operator-=(const Vector3<T>& rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;

        return *this;
    }

    Vector3<T>& operator*=(const T rhs)
    {
        x *= rhs;
        y *= rhs;
        z *= rhs;

        return *this;
    }

    Vector3<T>& operator/=(const T rhs)
    {
        x /= rhs;
        y /= rhs;
        z /= rhs;

        return *this;
    }

    Vector3<T> operator+(const Vector3<T>& rhs) const
    {
        return { x + rhs.x, y + rhs.y, z + rhs.z };
    }

    Vector3<T> operator-() const
    {
        return { -x, -y, -z };
    }

    Vector3<T> operator-(const Vector3<T>& rhs) const
    {
        return { x - rhs.x, y - rhs.y, z - rhs.z };
    }

    Vector3<T> operator*(const Vector3<T>& rhs) const
    {
        return { x * rhs.x, y * rhs.y, z * rhs.z };
    }

    Vector3<T> operator*(const T rhs) const
    {
        return { x * rhs, y * rhs, z * rhs };
    }

    Vector3<T> operator/(const T rhs) const
    {
        return { x / rhs, y / rhs, z / rhs };
    }

public:
    union
    {
        T v[3];
        struct
        {
            T x, y, z;
        };
    };
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

    return { x, y, z };
}

template <typename T>
Vector3<T> normalize(const Vector3<T>& v)
{
    return v / length(v);
}

template <typename T>
std::tuple<Vector3<T>, Vector3<T>> getBasis(const Vector3<T>& v)
{
    Vector3<T> v1 = std::abs(v.x) > std::abs(v.y)
            ? normalize(Vector3<T>(-v.z, 0.0f, v.x))
            : normalize(Vector3<T>(0.0f, v.z, -v.y));
    Vector3<T> v2 = cross(v, v1);
    return { v1, v2 };
}

using float3 = Vector3<float>;
using uint3 = Vector3<uint32_t>;
using byte3 = Vector3<uint8_t>;

template <typename T>
class Vector4
{
public:
    Vector4() { x = y = z = w = 0; }
    Vector4(T x, T y, T z, T w) : v{ x, y, z, w } {}
    Vector4(T v) : v{ v, v, v, v } {}
    Vector4(const Vector4<T>& v) : v{ v.x, v.y, v.z, v.w } {}
    Vector4(const T v[4]) : v{ v[0], v[1], v[2], v[3] } {}

    Vector4<T>& operator=(const Vector4<T>& rhs)
    {
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
        w = rhs.w;

        return *this;
    }

    Vector4<T> operator+(const Vector4<T>& rhs) const
    {
        return { x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w };
    }

    Vector4<T> operator-(const Vector4<T>& rhs) const
    {
        return { x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w };
    }

    Vector4<T> operator*(const T rhs) const
    {
        return { x * rhs, y * rhs, z * rhs, w * rhs };
    }

    Vector4<T> operator/(const T rhs) const
    {
        return { x / rhs, y / rhs, z / rhs, w / rhs };
    }

    Vector3<T> xyz() const
    {
        return Vector3<T>(x, y, z);
    }

public:
    union
    {
        T v[4];
        struct
        {
            T x, y, z, w;
        };
    };
};

template <typename T>
T dot(const Vector4<T>& v0, const Vector4<T>& v1)
{
    return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z + v0.w * v1.w;
}

using float4 = Vector4<float>;
}