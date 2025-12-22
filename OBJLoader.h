#pragma once
#include <string>
#include <vector>
#include "Vec3.h"

struct Vertex
{
    Vec3 position;
    Vec3 normal;
    Vertex() = default;
    Vertex(const Vec3& p, const Vec3& n) : position(p), normal(n) {}
};

struct Triangle
{
    Vertex v0, v1, v2;
    Vec3 faceNormal;

    Triangle(const Vertex& _v0, const Vertex& _v1, const Vertex& _v2)
        : v0(_v0), v1(_v1), v2(_v2)
    {
        Vec3 u = v1.position - v0.position;
        Vec3 v = v2.position - v0.position;
        faceNormal = Vec3::cross(u, v).normalized();
    }
};

class Mesh
{
public:
    bool loadFromOBJ(const std::string& filename, double targetBoundingBoxSize);
    void draw() const;
    bool isLoaded() const { return !m_triangles.empty(); }

private:
    std::vector<Triangle> m_triangles;
};