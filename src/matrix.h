#pragma once

#include "vector.h"

#include <cassert>

namespace alpine {
template <typename T>
class Matrix3x3
{
public:
    Matrix3x3()
        : v{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}
    {}

    T& operator[](uint32_t i)
    {
        return v[i];
    }

    const T& operator[](uint32_t i) const
    {
        return v[i];
    }

    Matrix3x3<T>& operator=(const Matrix3x3<T>& m)
    {
        memcpy(v, m.v, sizeof(T) * 9);
        return *this;
    }

    void setColumn(uint32_t i, const Vector3<T>& col)
    {
        for (uint32_t j = 0; j < 3; ++j)
        {
            v[3 * i + j] = col[j];
        }
    }

    const Vector3<T> getColumn(uint32_t i) const
    {
        return { v[3 * i], v[3 * i + 1], v[3 * i + 2] };
    }

private:
    T v[9];
};

template <typename T>
Vector3<T> mul(const Matrix3x3<T>& m, const Vector3<T>& v)
{
    return m.getColumn(0) * v[0] + m.getColumn(1) * v[1] + m.getColumn(2) * v[2];
}

template <typename T>
Matrix3x3<T> mul(const Matrix3x3<T>& m0, const Matrix3x3<T>& m1)
{
    Matrix3x3<T> m;
    m.setColumn(0, mul(m0, m1.getColumn(0)));
    m.setColumn(1, mul(m0, m1.getColumn(1)));
    m.setColumn(2, mul(m0, m1.getColumn(2)));

    return m;
}

using float3x3 = Matrix3x3<float>;

template <typename T>
class Matrix4x4
{
public:
    Matrix4x4()
    : v{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 }
    {}

    T& operator[](uint32_t i)
    {
        return v[i];
    }

    const T& operator[](uint32_t i) const
    {
        return v[i];
    }

    Matrix4x4<T>& operator=(const Matrix4x4<T>& m)
    {
        memcpy(v, m.v, sizeof(T) * 16);
        return *this;
    }

    void setColumn(uint32_t i, const Vector4<T>& col)
    {
        for (uint32_t j = 0; j < 4; ++j)
        {
            v[4 * i + j] = col[j];
        }
    }

    const Vector4<T> getColumn(uint32_t i) const
    {
        return { v[4 * i], v[4 * i + 1], v[4 * i + 2], v[4 * i + 3] };
    }

private:
    T v[16];
};

template <typename T>
Vector4<T> mul(const Matrix4x4<T>& m, const Vector4<T>& v)
{
    return  m.getColumn(0) * v[0] + m.getColumn(1) * v[1] +
        m.getColumn(2) * v[2] + m.getColumn(3) * v[3];
}

template <typename T>
Matrix4x4<T> mul(const Matrix4x4<T>& m0, const Matrix4x4<T>& m1)
{
    Matrix4x4<T> m;
    m.setColumn(0, mul(m0, m1.getColumn(0)));
    m.setColumn(1, mul(m0, m1.getColumn(1)));
    m.setColumn(2, mul(m0, m1.getColumn(2)));
    m.setColumn(3, mul(m0, m1.getColumn(3)));

    return m;
}

using float4x4 = Matrix4x4<float>;

}