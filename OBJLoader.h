// OBJLoader.h
#pragma once
#include <vector>
#include <string>
#include "Vec3.h"

class Mesh
{
public:
    bool loadFromOBJ(const std::string& path, double scale = 1.0);
    void draw() const;

private:
    struct Tri
    {
        Vec3 v[3];
        Vec3 n[3];
    };
    std::vector<Tri> m_tris;
};
