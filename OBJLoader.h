#pragma once
#include <vector>
#include <string>
#include "Vec3.h"

// Simple vertex with position and normal
struct Vertex
{
    Vec3 position;
    Vec3 normal;

    Vertex() {}
    Vertex(const Vec3& p, const Vec3& n)
        : position(p), normal(n) {}
};

// Triangle made of 3 vertices and a precomputed face normal
struct Triangle
{
    Vertex v0;
    Vertex v1;
    Vertex v2;
    Vec3   faceNormal;  // used if per-vertex normals are missing

    Triangle() {}

    Triangle(const Vertex& a, const Vertex& b, const Vertex& c)
        : v0(a), v1(b), v2(c)
    {
        computeFaceNormal();
    }

    void computeFaceNormal()
    {
        Vec3 e1 = v1.position - v0.position;
        Vec3 e2 = v2.position - v0.position;
        faceNormal = Vec3::cross(e1, e2).normalized();
    }
};

// Mesh class used by main.cpp
class Mesh
{
public:
    // Note: name is m_triangles to match any existing usage
    std::vector<Triangle> m_triangles;

    Mesh() {}

    // Draw with OpenGL
    void draw() const;

    // Load OBJ file with a uniform scale
    bool loadFromOBJ(const std::string& filename, double scale);
};
