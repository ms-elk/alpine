#pragma once

#include "vector.h"

#include <cassert>

namespace alpine {
template <typename T>
class Matrix3x3
{
public:
    Matrix3x3() {}
    Matrix3x3(const Vector3<T>& v0, const Vector3<T>& v1, const Vector3<T>& v2)
        : v{ v0, v1, v2 }
    {}

    Vector3<T>& operator[](uint32_t i)
    {
        return v[i];
    }

    const Vector3<T>& operator[](uint32_t i) const
    {
        return v[i];
    }

    Matrix3x3<T>& operator=(const Matrix3x3<T>& m)
    {
        v[0] = m.v[0];
        v[1] = m.v[1];
        v[2] = m.v[2];

        return *this;
    }

    void setColumn(uint32_t i, const Vector3<T>& col)
    {
        v[0][i] = col[0];
        v[1][i] = col[1];
        v[2][i] = col[2];
    }

    const Vector3<T> getColumn(uint32_t i) const
    {
        return { v[0][i], v[1][i], v[2][i] };
    }

private:
    Vector3<T> v[3];
};

template <typename T>
Vector3<T> mul(const Matrix3x3<T>& m, const Vector3<T>& v)
{
    return { dot(m[0], v), dot(m[1], v), dot(m[2], v) };
}

template <typename T>
Matrix3x3<T> mul(const Matrix3x3<T>& m0, const Matrix3x3<T>& m1)
{
    return {
        Vector3<T>(dot(m0[0], m1.getColumn(0)), dot(m0[0], m1.getColumn(1)), dot(m0[0], m1.getColumn(2))),
        Vector3<T>(dot(m0[1], m1.getColumn(0)), dot(m0[1], m1.getColumn(1)), dot(m0[1], m1.getColumn(2))),
        Vector3<T>(dot(m0[2], m1.getColumn(0)), dot(m0[2], m1.getColumn(1)), dot(m0[2], m1.getColumn(2)))
    };
}

using float3x3 = Matrix3x3<float>;

}