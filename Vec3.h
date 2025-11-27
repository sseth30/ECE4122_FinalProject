/*
Author: Your Name
Class: ECE4122 or ECE6122
Last Date Modified: 11/26/2025
Description:
Simple 3D vector type with basic operations.
*/

#pragma once
#include <cmath>

struct Vec3
{
    double x;
    double y;
    double z;

    Vec3(double xx = 0.0, double yy = 0.0, double zz = 0.0)
        : x(xx), y(yy), z(zz) {}

    Vec3 operator+(const Vec3& rhs) const
    {
        return Vec3(x + rhs.x, y + rhs.y, z + rhs.z);
    }

    Vec3 operator-(const Vec3& rhs) const
    {
        return Vec3(x - rhs.x, y - rhs.y, z - rhs.z);
    }

    Vec3 operator*(double s) const
    {
        return Vec3(x * s, y * s, z * s);
    }

    Vec3 operator/(double s) const
    {
        return Vec3(x / s, y / s, z / s);
    }

    Vec3& operator+=(const Vec3& rhs)
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    double length() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }

    Vec3 normalized() const
    {
        double len = length();
        if (len > 0.0)
            return *this / len;
        return Vec3(0.0, 0.0, 0.0);
    }

    static Vec3 cross(const Vec3& a, const Vec3& b)
    {
        return Vec3(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        );
    }

    static double dot(const Vec3& a, const Vec3& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
};
