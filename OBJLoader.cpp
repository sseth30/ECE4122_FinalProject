/*
Author: <your name>
Class: ECE4122 or ECE6122
Last Date Modified: <date>

Description:
 Implementation of the simple Wavefront OBJ loader used to render the UAV mesh.
*/

#include "OBJLoader.h"

#include <fstream>
#include <sstream>
#include <iostream>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

bool Mesh::loadFromOBJ(const std::string& filePath, double targetBoundingBoxSize)
{
    m_triangles.clear();

    std::ifstream file(filePath.c_str());
    if (!file)
    {
        std::cerr << "Failed to open OBJ file: " << filePath << std::endl;
        return false;
    }

    std::vector<Vec3> vertices;
    std::vector<Vec3> normals;

    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty())
        {
            continue;
        }

        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;

        if (prefix == "v")
        {
            double x, y, z;
            iss >> x >> y >> z;
            vertices.emplace_back(x, y, z);
        }
        else if (prefix == "vn")
        {
            double x, y, z;
            iss >> x >> y >> z;
            normals.emplace_back(x, y, z);
        }
        else if (prefix == "f")
        {
            // Faces can have 3 or more vertices. We will read all indices for this face
            // and triangulate fan style if needed.
            std::vector<int> vIdx;
            std::vector<int> nIdx;

            std::string token;
            while (iss >> token)
            {
                // Token formats: v, v//n, v/t/n, v/t
                int vi = 0;
                int ni = 0;

                size_t firstSlash = token.find('/');
                size_t secondSlash = std::string::npos;
                if (firstSlash != std::string::npos)
                {
                    secondSlash = token.find('/', firstSlash + 1);
                }

                if (firstSlash == std::string::npos)
                {
                    // Only vertex index.
                    vi = std::stoi(token);
                }
                else
                {
                    std::string vPart = token.substr(0, firstSlash);
                    vi = std::stoi(vPart);

                    if (secondSlash != std::string::npos)
                    {
                        std::string nPart = token.substr(secondSlash + 1);
                        if (!nPart.empty())
                        {
                            ni = std::stoi(nPart);
                        }
                    }
                }

                vIdx.push_back(vi);
                nIdx.push_back(ni);
            }

            if (vIdx.size() < 3)
            {
                continue;
            }

            // Triangulate the polygon into fan of triangles.
            for (size_t i = 1; i + 1 < vIdx.size(); ++i)
            {
                int idx0 = vIdx[0];
                int idx1 = vIdx[i];
                int idx2 = vIdx[i + 1];

                Vec3 v0 = vertices[static_cast<size_t>(idx0 - 1)];
                Vec3 v1 = vertices[static_cast<size_t>(idx1 - 1)];
                Vec3 v2 = vertices[static_cast<size_t>(idx2 - 1)];

                Vec3 normal;
                int n0 = nIdx[0];
                if (!normals.empty() && n0 > 0 && static_cast<size_t>(n0 - 1) < normals.size())
                {
                    normal = normals[static_cast<size_t>(n0 - 1)].normalized();
                }
                else
                {
                    normal = Vec3::cross(v1 - v0, v2 - v0).normalized();
                }

                Triangle tri;
                tri.v0 = v0;
                tri.v1 = v1;
                tri.v2 = v2;
                tri.normal = normal;
                m_triangles.push_back(tri);
            }
        }
    }

    if (m_triangles.empty())
    {
        std::cerr << "No geometry loaded from OBJ file: " << filePath << std::endl;
        return false;
    }

    // Compute bounding box and scale if requested.
    if (targetBoundingBoxSize > 0.0)
    {
        Vec3 minPt(1e9, 1e9, 1e9);
        Vec3 maxPt(-1e9, -1e9, -1e9);

        for (const Triangle& tri : m_triangles)
        {
            const Vec3 verts[3] = { tri.v0, tri.v1, tri.v2 };
            for (int i = 0; i < 3; ++i)
            {
                const Vec3& v = verts[i];
                if (v.x < minPt.x) minPt.x = v.x;
                if (v.y < minPt.y) minPt.y = v.y;
                if (v.z < minPt.z) minPt.z = v.z;

                if (v.x > maxPt.x) maxPt.x = v.x;
                if (v.y > maxPt.y) maxPt.y = v.y;
                if (v.z > maxPt.z) maxPt.z = v.z;
            }
        }

        Vec3 size = maxPt - minPt;
        double maxDim = std::max(size.x, std::max(size.y, size.z));
        if (maxDim > 0.0)
        {
            double scale = targetBoundingBoxSize / maxDim;
            Vec3 center = (minPt + maxPt) * 0.5;

            for (Triangle& tri : m_triangles)
            {
                tri.v0 = (tri.v0 - center) * scale;
                tri.v1 = (tri.v1 - center) * scale;
                tri.v2 = (tri.v2 - center) * scale;
                tri.normal = tri.normal.normalized();
            }
        }
    }

    std::cout << "Loaded OBJ '" << filePath << "' with " << m_triangles.size()
              << " triangles." << std::endl;

    return true;
}

void Mesh::draw() const
{
    glBegin(GL_TRIANGLES);
    for (const Triangle& tri : m_triangles)
    {
        glNormal3d(tri.normal.x, tri.normal.y, tri.normal.z);

        glVertex3d(tri.v0.x, tri.v0.y, tri.v0.z);
        glVertex3d(tri.v1.x, tri.v1.y, tri.v1.z);
        glVertex3d(tri.v2.x, tri.v2.y, tri.v2.z);
    }
    glEnd();
}
