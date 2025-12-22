#pragma once
#include <cmath>

class Vec3
{
public:
    double x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

    Vec3 operator+(const Vec3& rhs) const { return Vec3(x+rhs.x, y+rhs.y, z+rhs.z); }
    Vec3 operator-(const Vec3& rhs) const { return Vec3(x-rhs.x, y-rhs.y, z-rhs.z); }
    Vec3 operator*(double s) const { return Vec3(x*s, y*s, z*s); }
    Vec3 operator/(double s) const { return Vec3(x/s, y/s, z/s); }
    
    Vec3& operator+=(const Vec3& rhs) { x+=rhs.x; y+=rhs.y; z+=rhs.z; return *this; }
    
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    
    Vec3 normalized() const {
        double l = length();
        return l > 0 ? (*this)/l : *this;
    }

    static double dot(const Vec3& a, const Vec3& b) {
        return a.x*b.x + a.y*b.y + a.z*b.z;
    }

    static Vec3 cross(const Vec3& a, const Vec3& b) {
        return Vec3(a.y*b.z - a.z*b.y,
                    a.z*b.x - a.x*b.z,
                    a.x*b.y - a.y*b.x);
    }
};