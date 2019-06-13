#pragma once
#ifndef CORE_HPP_
#define CORE_HPP_

#include "core/precision.hpp"
#include <math.h>

namespace chaos {
class Vector3 {
public:
    real x;
    real y;
    real z;

private:
    real pad;

public:
    Vector3()
        : x(0)
        , y(0)
        , z(0)
    {
    }
    Vector3(const real x, const real y, const real z)
        : x(x)
        , y(y)
        , z(z)
    {
    }

    void invert()
    {
        x = -x;
        y = -y;
        z = -z;
    }

    real magnitude() const
    {
        return sqrt(x * x + y * y + z * z);
    }

    real squareMagnitude() const
    {
        return x * x + y * y + z * z;
    }

    void normalise()
    {
        real l = magnitude();
        if (l > 0) {
            (*this) *= ((real)1) / l;
        }
    }

    void operator*=(const real value)
    {
        x *= value;
        y *= value;
        z *= value;
    }

    Vector3 operator*(const real value) const
    {
        return Vector3(x * value, y * value, z * value);
    }

    void operator+=(const Vector3& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
    }

    Vector3 operator+(const Vector3& v) const
    {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }

    void operator-=(const Vector3& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
    }

    Vector3 operator-(const Vector3& v) const
    {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }

    void addScaledVector(const Vector3& vector, real scale)
    {
        x += vector.x * scale;
        y += vector.y * scale;
        z += vector.z * scale;
    }

    Vector3 componentProduct(const Vector3& vector) const
    {
        return Vector3(x * vector.x, y * vector.y, z * vector.z);
    }

    void componentProductUpdate(const Vector3& vector)
    {
        x *= vector.x;
        y *= vector.y;
        z *= vector.z;
    }

    real scalarProduct(const Vector3& vector) const
    {
        return x * vector.x + y * vector.y + z * vector.z;
    }

    real operator*(const Vector3& vector) const
    {
        return x * vector.x + y * vector.y + z * vector.z;
    }

    Vector3 vectorProduct(const Vector3& vector) const
    {
        return Vector3(y * vector.z - z * vector.y, z * vector.x - x * vector.z, x * vector.y - y * vector.x);
    }

    void operator%=(const Vector3& vector)
    {
        *this = vectorProduct(vector);
    }

    Vector3 operator%(const Vector3& vector) const
    {
        return Vector3(y * vector.z - z * vector.y, z * vector.x - x * vector.z, x * vector.y - y * vector.x);
    }

    void makeOrthonormalBasis(Vector3* a, Vector3* b, Vector3* c)
    {
        a->normalise();
        (*c) = (*a) % (*b);
        if (c.squareMagnitude() == 0.0)
            return;
        c->normalise();
        (*b) = (*c) % (*a);
    }

    void clear()
    {
        x = y = z = 0;
    }
};

#endif // CORE_HPP_