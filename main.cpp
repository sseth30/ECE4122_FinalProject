/*
Author: Your Name
Class: ECE4122 or ECE6122
Last Date Modified: 11/26/2025

Description:
 Main entry point for the ECE 4122 / 6122 final project.

 This program creates a 3D OpenGL simulation of 15 UAVs:
   1) UAVs start on the football field at the 0/25/50/25/0 yard lines.
   2) Each UAV takes off and flies to a rendezvous point above midfield
      using a PID controller (Appendix B).
   3) Once all UAVs reach the rendezvous region, they transition to
      flying along the surface of a virtual wireframe sphere.
*/

#include <iostream>
#include <vector>
#include <memory>
#include <chrono>
#include <cmath>
#include <fstream>
#include <string>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/freeglut.h>
#endif

#ifndef GL_BGR
#define GL_BGR 0x80E0
#endif

#include "Vec3.h"
#include "ECE_UAV.h"
#include "OBJLoader.h"

// Window
static const int   kWindowWidth  = 800;
static const int   kWindowHeight = 600;

// Field dimensions (meters; approximate)
static const float kFieldLength  = 100.0f;
static const float kFieldWidth   = 80.0f;

static const int   kNumUAVs      = 15;

// Global state
static std::vector<std::unique_ptr<ECE_UAV>> g_uavs;
static Mesh    g_uavMesh;
static bool    g_haveMesh = false;

static GLuint  g_fieldTexture = 0;
static bool    g_haveTexture  = false;

static std::chrono::time_point<std::chrono::steady_clock> g_simStartTime;

// Forward declarations
void displayCallback();
void reshapeCallback(int width, int height);
void timerCallback(int value);

// --------------------------------------------------------------
// BMP loader (24-bit)
// --------------------------------------------------------------
GLuint loadBMPTexture(const char* filename)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file)
    {
        std::cerr << "Error: Could not open texture file "
                  << filename << std::endl;
        return 0;
    }

    unsigned char header[54];
    if (!file.read(reinterpret_cast<char*>(header), 54))
        return 0;

    if (header[0] != 'B' || header[1] != 'M')
        return 0;

    unsigned int dataPos   = *reinterpret_cast<unsigned int*>(&header[0x0A]);
    unsigned int imageSize = *reinterpret_cast<unsigned int*>(&header[0x22]);
    unsigned int width     = *reinterpret_cast<unsigned int*>(&header[0x12]);
    unsigned int height    = *reinterpret_cast<unsigned int*>(&header[0x16]);

    if (imageSize == 0)
        imageSize = width * height * 3;
    if (dataPos == 0)
        dataPos = 54;

    std::vector<unsigned char> data(imageSize);
    file.seekg(dataPos);
    file.read(reinterpret_cast<char*>(data.data()), imageSize);
    file.close();

    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                 width, height, 0,
                 GL_BGR, GL_UNSIGNED_BYTE,
                 data.data());

    return textureID;
}

// --------------------------------------------------------------
// UAV initialization: arrange at 0,25,50,25,0 yard lines
// --------------------------------------------------------------
void initUAVs()
{
    g_uavs.clear();

    // Yard to meter conversion
    const double yardToMeter = 0.9144;

    // 0, 25, 50, 25, 0 yard lines from one end zone
    // Center of field is at 50 yard line → y=0.
    // So 0 yard line = +50 yards, 25 = +25, 50 = 0, etc.
    std::vector<double> yardLines = { 0.0, 25.0, 50.0, 75.0, 100.0 };
    std::vector<double> yLines;

    for (double y : yardLines)
    {
        double offsetFrom50 = (y - 50.0) * yardToMeter;
        yLines.push_back(offsetFrom50);
    }

    int idCounter = 0;
    for (double y : yLines)
    {
        double xOffsets[] = { -15.0, 0.0, 15.0 };  // 3 per line

        for (double x : xOffsets)
        {
            Vec3 startPos(x, y, 0.0);
            auto uav = std::make_unique<ECE_UAV>(idCounter, startPos, kNumUAVs);
            uav->start();
            g_uavs.push_back(std::move(uav));
            ++idCounter;
        }
    }
}

// --------------------------------------------------------------
// Draw textured field quad
// --------------------------------------------------------------
void drawField()
{
    GLfloat mat_ambient[]  = { 0.4f, 0.4f, 0.4f, 1.0f };
    GLfloat mat_diffuse[]  = { 0.9f, 0.9f, 0.9f, 1.0f };
    GLfloat mat_specular[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    glMaterialfv(GL_FRONT, GL_AMBIENT,  mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE,  mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);

    if (g_haveTexture)
    {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, g_fieldTexture);
        glColor3f(1.0f, 1.0f, 1.0f);
    }
    else
    {
        glDisable(GL_TEXTURE_2D);
        glColor3f(0.0f, 0.6f, 0.0f);
    }

    float hw = kFieldWidth  * 0.5f;
    float hl = kFieldLength * 0.5f;

    glBegin(GL_QUADS);
    glNormal3f(0.0f, 0.0f, 1.0f);

    // Straight mapping: image X→world X, image Y→world Y
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-hw, -hl, 0.0f); // bottom-left
    glTexCoord2f(1.0f, 0.0f); glVertex3f( hw, -hl, 0.0f); // bottom-right
    glTexCoord2f(1.0f, 1.0f); glVertex3f( hw,  hl, 0.0f); // top-right
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-hw,  hl, 0.0f); // top-left

    glEnd();
    glDisable(GL_TEXTURE_2D);
}

// --------------------------------------------------------------
// Draw virtual sphere (wireframe)
// --------------------------------------------------------------
void drawVirtualSphere()
{
    glDisable(GL_TEXTURE_2D);

    glPushMatrix();
    glTranslatef(0.0f, 0.0f, 40.0f); // must match center in ECE_UAV.cpp
    glColor3f(0.7f, 0.7f, 0.9f);
    glutWireSphere(12.0, 24, 24);   // radius must match kSphereRadius
    glPopMatrix();
}

// --------------------------------------------------------------
// Display callback
// --------------------------------------------------------------
void displayCallback()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Camera
    gluLookAt(0.0, -160.0, 90.0,
              0.0,    0.0, 20.0,
              0.0,    0.0,  1.0);

    GLfloat light_pos[] = { 60.0f, -40.0f, 120.0f, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

    drawField();
    drawVirtualSphere();

    // Draw UAVs (torus mesh or red spheres)
    for (const auto& uav : g_uavs)
    {
        Vec3 pos, vel;
        uav->getState(pos, vel);

        glPushMatrix();
        glTranslatef(static_cast<float>(pos.x),
                     static_cast<float>(pos.y),
                     static_cast<float>(pos.z));

        if (g_haveMesh)
        {
            GLfloat mat_col[] = { 0.9f, 0.3f, 0.3f, 1.0f };
            glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_col);
            g_uavMesh.draw();
        }
        else
        {
            GLfloat mat_col[] = { 0.9f, 0.1f, 0.1f, 1.0f };
            glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_col);
            glutSolidSphere(0.6, 16, 16);
        }

        glPopMatrix();
    }

    glutSwapBuffers();
}

// --------------------------------------------------------------
// Timer callback: collision checks + redraw
// --------------------------------------------------------------
void timerCallback(int)
{
    // Very simple collision handling: swap velocities when too close
    for (size_t i = 0; i < g_uavs.size(); ++i)
    {
        for (size_t j = i + 1; j < g_uavs.size(); ++j)
        {
            Vec3 p1, v1;
            Vec3 p2, v2;
            g_uavs[i]->getState(p1, v1);
            g_uavs[j]->getState(p2, v2);

            double dist = (p1 - p2).length();
            if (dist < 0.21)   // 20 cm box + 1 cm threshold
            {
                g_uavs[i]->setVelocity(v2);
                g_uavs[j]->setVelocity(v1);
            }
        }
    }

    glutPostRedisplay();
    glutTimerFunc(30, timerCallback, 0); // ~33 FPS
}

// --------------------------------------------------------------
// Reshape
// --------------------------------------------------------------
void reshapeCallback(int width, int height)
{
    if (height == 0) height = 1;
    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0,
                   static_cast<double>(width) /
                   static_cast<double>(height),
                   1.0, 500.0);
    glMatrixMode(GL_MODELVIEW);
}

// --------------------------------------------------------------
// OpenGL init
// --------------------------------------------------------------
void initOpenGL()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);

    GLfloat ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
    GLfloat diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);

    glClearColor(0.05f, 0.05f, 0.10f, 1.0f);
}

// --------------------------------------------------------------
// main
// --------------------------------------------------------------
int main(int argc, char** argv)
{
    std::string objPath = "Torus.obj";
    if (argc >= 2)
        objPath = argv[1];

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(kWindowWidth, kWindowHeight);
    glutCreateWindow("ECE 4122 Final Project - UAV Show");

    initOpenGL();

    // Load UAV mesh (torus)
    g_haveMesh = g_uavMesh.loadFromOBJ(objPath, 0.5);

    // Load football field texture
    g_fieldTexture = loadBMPTexture("ff.bmp");
    if (g_fieldTexture != 0)
        g_haveTexture = true;

    // Initialize UAVs and start their threads
    initUAVs();
    g_simStartTime = std::chrono::steady_clock::now();

    // Callbacks
    glutDisplayFunc(displayCallback);
    glutReshapeFunc(reshapeCallback);
    glutTimerFunc(30, timerCallback, 0);

    glutMainLoop();

    return 0;
}
