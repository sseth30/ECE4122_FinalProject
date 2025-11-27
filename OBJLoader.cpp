#include "OBJLoader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm> 
#include <string>      // <-- and this

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

bool Mesh::loadFromOBJ(const std::string& filename, double scale)
{
    std::ifstream file(filename);
    if (!file)
    {
        std::cerr << "Failed to open OBJ file: " << filename << std::endl;
        return false;
    }

    std::vector<Vec3> positions;
    std::vector<Vec3> normals;

    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty() || line[0] == '#')
            continue;

        std::istringstream iss(line);
        std::string tag;
        iss >> tag;

        if (tag == "v")
        {
            double x, y, z;
            iss >> x >> y >> z;
            positions.emplace_back(x * scale, y * scale, z * scale);
        }
        else if (tag == "vn")
        {
            double x, y, z;
            iss >> x >> y >> z;
            normals.emplace_back(x, y, z);
        }
        else if (tag == "f")
        {
            // We will support faces in these forms:
            //  f v1 v2 v3
            //  f v1//n1 v2//n2 v3//n3
            //  f v1/t1/n1 v2/t2/n2 v3/t3/n3
            // We will ignore texture coordinates and only care about v and vn.

            std::string f1, f2, f3;
            iss >> f1 >> f2 >> f3;
            if (f1.empty() || f2.empty() || f3.empty())
                continue;

            auto parseIndex = [](const std::string& token, int& vi, int& ni)
{
            // token can be: "v", "v//n", or "v/t/n"
            vi = -1;
            ni = -1;

            // Count slashes
            int slashCount = 0;
            for (char c : token)
                if (c == '/')
                    ++slashCount;

            if (slashCount == 0)
            {
                // Just "v"
                vi = std::stoi(token) - 1;
                return;
            }

            // General: split by '/'
            std::stringstream s(token);
            std::string a, b, c;
            std::getline(s, a, '/'); // v
            std::getline(s, b, '/'); // t (ignored)
            std::getline(s, c, '/'); // n

            if (!a.empty())
                vi = std::stoi(a) - 1;
            if (!c.empty())
                ni = std::stoi(c) - 1;
        };



            int vIdx[3] = { -1, -1, -1 };
            int nIdx[3] = { -1, -1, -1 };

            parseIndex(f1, vIdx[0], nIdx[0]);
            parseIndex(f2, vIdx[1], nIdx[1]);
            parseIndex(f3, vIdx[2], nIdx[2]);

            bool valid = true;
            for (int i = 0; i < 3; ++i)
            {
                if (vIdx[i] < 0 || vIdx[i] >= static_cast<int>(positions.size()))
                    valid = false;
            }
            if (!valid) continue;

            // Build triangle vertices
            Vertex verts[3];
            for (int i = 0; i < 3; ++i)
            {
                Vec3 pos = positions[vIdx[i]];
                Vec3 nrm;

                if (nIdx[i] >= 0 && nIdx[i] < static_cast<int>(normals.size()))
                    nrm = normals[nIdx[i]];
                else
                    nrm = Vec3(0.0, 0.0, 0.0); // we will fix with face normal if needed

                verts[i] = Vertex(pos, nrm);
            }

            Triangle tri(verts[0], verts[1], verts[2]);

            // If normals were all zero, use faceNormal for each vertex
            if (verts[0].normal.length() < 1e-6 &&
                verts[1].normal.length() < 1e-6 &&
                verts[2].normal.length() < 1e-6)
            {
                tri.v0.normal = tri.faceNormal;
                tri.v1.normal = tri.faceNormal;
                tri.v2.normal = tri.faceNormal;
            }

            m_triangles.push_back(tri);
        }
    }

    std::cout << "Loaded OBJ '" << filename
              << "' with " << m_triangles.size() << " triangles.\n";
    return !m_triangles.empty();
}

void Mesh::draw() const
{
    glBegin(GL_TRIANGLES);
    for (const auto& tri : m_triangles)
    {
        // Use per-vertex normals if available, otherwise face normal
        Vec3 n0 = tri.v0.normal.length() > 1e-6 ? tri.v0.normal : tri.faceNormal;
        Vec3 n1 = tri.v1.normal.length() > 1e-6 ? tri.v1.normal : tri.faceNormal;
        Vec3 n2 = tri.v2.normal.length() > 1e-6 ? tri.v2.normal : tri.faceNormal;

        glNormal3d(n0.x, n0.y, n0.z);
        glVertex3d(tri.v0.position.x, tri.v0.position.y, tri.v0.position.z);

        glNormal3d(n1.x, n1.y, n1.z);
        glVertex3d(tri.v1.position.x, tri.v1.position.y, tri.v1.position.z);

        glNormal3d(n2.x, n2.y, n2.z);
        glVertex3d(tri.v2.position.x, tri.v2.position.y, tri.v2.position.z);
    }
    glEnd();
}
