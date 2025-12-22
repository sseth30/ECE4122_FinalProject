/*
Author: Satchit Seth
Class: ECE4122
Last Date Modified: 12/01/2025

Description:
 15 UAVs:
   Start on the football field at 0, 25, 50, 25, 0 yard lines (5 rows of 3)
   Sit for 5 s, then fly to (0,0,50) using PID
   When they hit the virtual sphere surface (radius 10 m, center (0,0,50))
   they move along the surface with speed 2–10 m/s
   After all are on the sphere, they continue for 60 s, then freeze

 This file implements an OBJ mesh loader and draw routine used to render
 the 3D UAV model in the simulation.
*/

#include "OBJLoader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm> 
#include <string>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

/*
 * Function   : Mesh::loadFromOBJ
 * Purpose    : Load a triangle mesh from a Wavefront OBJ file, optionally
 *              scaling it to fit inside a bounding box of a given size.
 * Input      : filename             - path to the OBJ file to load.
 *              targetBoundingBoxSize - if > 0, the loaded mesh is uniformly
 *                                      scaled so its largest dimension equals
 *                                      this size.
 * Output     : Populates m_triangles with triangle data for rendering.
 * Return     : true  - if at least one triangle was loaded successfully.
 *              false - if the file cannot be opened or no valid triangles.
 */
bool Mesh::loadFromOBJ(const std::string& filename, double targetBoundingBoxSize)
{
    // Clear any existing triangles before loading.
    m_triangles.clear();

    std::ifstream file(filename);
    if (!file)
    {
        std::cerr << "Failed to open OBJ file: " << filename << std::endl;
        return false;
    }

    // Temporary storage for vertex positions and normals.
    std::vector<Vec3> positions;
    std::vector<Vec3> normals;

    std::string line;
    while (std::getline(file, line))
    {
        // Skip empty lines and comments.
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        std::string tag;
        iss >> tag;

        // "v x y z" = vertex position.
        if (tag == "v")
        {
            double x, y, z;
            iss >> x >> y >> z;
            positions.emplace_back(x, y, z);
        }
        // "vn x y z" = vertex normal.
        else if (tag == "vn")
        {
            double x, y, z;
            iss >> x >> y >> z;
            normals.emplace_back(x, y, z);
        }
        // "f ..." = face (triangle in this loader).
        else if (tag == "f")
        {
            // Expect three vertices per face (triangle).
            std::string f1, f2, f3;
            iss >> f1 >> f2 >> f3;
            if (f1.empty() || f2.empty() || f3.empty()) continue;

            // Helper lambda to parse indices from tokens like:
            // "v", "v/t", "v//n", or "v/t/n".
            auto parseIndex = [](const std::string& token, int& vi, int& ni)
            {
                vi = -1; 
                ni = -1;

                size_t firstSlash = token.find('/');
                if (firstSlash == std::string::npos)
                {
                    // Format: "v"
                    vi = std::stoi(token) - 1;
                    return;
                }

                // Format starts with "v/...".
                vi = std::stoi(token.substr(0, firstSlash)) - 1;
                
                size_t secondSlash = token.find('/', firstSlash + 1);
                if (secondSlash != std::string::npos)
                {
                    // Format: "v/t/n" or "v//n"
                    std::string nStr = token.substr(secondSlash + 1);
                    if (!nStr.empty()) 
                        ni = std::stoi(nStr) - 1;
                }
            };

            int vIdx[3], nIdx[3];
            parseIndex(f1, vIdx[0], nIdx[0]);
            parseIndex(f2, vIdx[1], nIdx[1]);
            parseIndex(f3, vIdx[2], nIdx[2]);

            // Validate vertex indices before using them.
            bool valid = true;
            for (int i = 0; i < 3; ++i)
                if (vIdx[i] < 0 || vIdx[i] >= (int)positions.size()) 
                    valid = false;
            if (!valid) continue;

            // Build three vertices for this triangle.
            Vertex verts[3];
            for (int i = 0; i < 3; ++i)
            {
                verts[i].position = positions[vIdx[i]];

                // If we have a normal index, use it; otherwise default to (0,0,0)
                // and rely on the Triangle constructor to compute a flat normal.
                if (nIdx[i] >= 0 && nIdx[i] < (int)normals.size())
                    verts[i].normal = normals[nIdx[i]];
                else
                    verts[i].normal = Vec3(0, 0, 0);
            }

            // Store the triangle in the mesh.
            m_triangles.emplace_back(verts[0], verts[1], verts[2]);
        }
    }

    // Optional uniform scaling to fit the mesh in a box of size targetBoundingBoxSize.
    if (targetBoundingBoxSize > 0.0 && !m_triangles.empty())
    {
        // Compute bounding box of the current mesh.
        Vec3 minPt(1e9, 1e9, 1e9), maxPt(-1e9, -1e9, -1e9);
        for (const auto& tri : m_triangles)
        {
            for (const auto& v : { tri.v0, tri.v1, tri.v2 })
            {
                if (v.position.x < minPt.x) minPt.x = v.position.x;
                if (v.position.y < minPt.y) minPt.y = v.position.y;
                if (v.position.z < minPt.z) minPt.z = v.position.z;
                if (v.position.x > maxPt.x) maxPt.x = v.position.x;
                if (v.position.y > maxPt.y) maxPt.y = v.position.y;
                if (v.position.z > maxPt.z) maxPt.z = v.position.z;
            }
        }
        
        // Get the largest dimension of the bounding box.
        Vec3 size = maxPt - minPt;
        double maxDim = std::max({size.x, size.y, size.z});
        if (maxDim > 0)
        {
            // Compute scale factor to fit largest dimension to targetBoundingBoxSize.
            double scale = targetBoundingBoxSize / maxDim;
            Vec3 center = (minPt + maxPt) * 0.5;

            // Recenter and scale all triangle vertices.
            for (auto& tri : m_triangles)
            {
                tri.v0.position = (tri.v0.position - center) * scale;
                tri.v1.position = (tri.v1.position - center) * scale;
                tri.v2.position = (tri.v2.position - center) * scale;
            }
        }
    }

    std::cout << "Loaded " << filename << ": " << m_triangles.size() << " triangles.\n";
    return !m_triangles.empty();
}

/*
 * Function   : Mesh::draw
 * Purpose    : Render the loaded mesh as GL_TRIANGLES using immediate mode.
 *              Each triangle uses per-vertex normals if available, otherwise
 *              a flat face normal computed in Triangle.
 * Input      : None.
 * Output     : Issues OpenGL calls to draw the mesh.
 * Return     : None.
 */
void Mesh::draw() const
{
    glBegin(GL_TRIANGLES);
    for (const auto& tri : m_triangles)
    {
        // Select per-vertex normal if present, else fall back to face normal.
        Vec3 n0 = tri.v0.normal.length() > 0 ? tri.v0.normal : tri.faceNormal;
        Vec3 n1 = tri.v1.normal.length() > 0 ? tri.v1.normal : tri.faceNormal;
        Vec3 n2 = tri.v2.normal.length() > 0 ? tri.v2.normal : tri.faceNormal;

        glNormal3d(n0.x, n0.y, n0.z);
        glVertex3d(tri.v0.position.x, tri.v0.position.y, tri.v0.position.z);

        glNormal3d(n1.x, n1.y, n1.z);
        glVertex3d(tri.v1.position.x, tri.v1.position.y, tri.v1.position.z);

        glNormal3d(n2.x, n2.y, n2.z);
        glVertex3d(tri.v2.position.x, tri.v2.position.y, tri.v2.position.z);
    }
    glEnd();
}
