/*
Author: Your Name
Class: ECE4122
Last Date Modified: 11/26/2025

Description:
 15 UAVs:
   • Start on the football field at 0, 25, 50, 25, 0 yard lines (5 rows of 3)
   • Sit for 5 s, then fly to (0,0,50) using PID (in ECE_UAV.cpp)
   • When they hit the virtual sphere surface (radius 10 m, center (0,0,50))
     they move along the surface with speed 2–10 m/s
   • After all are on the sphere, they continue for 60 s, then freeze
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

// -----------------------------------------------------------------------------
// Window / field constants
// -----------------------------------------------------------------------------
static const int   kWindowWidth  = 1000;
static const int   kWindowHeight = 750;

// Field Dimensions (Meters)
// 120 yards total length (including endzones) ~ 109.7m
// 53.3 yards width ~ 48.7m
static const float kFieldLength  = 109.7f; 
static const float kFieldWidth   = 48.76f; 

static const int   kNumUAVs      = 15;

// -----------------------------------------------------------------------------
// Global state
// -----------------------------------------------------------------------------
static std::vector<std::unique_ptr<ECE_UAV>> g_uavs;

static GLuint g_fieldTexture = 0;
static bool   g_haveTexture  = false;

// -----------------------------------------------------------------------------
// Forward declarations
// -----------------------------------------------------------------------------
void displayCallback();
void reshapeCallback(int width, int height);
void timerCallback(int value);

// -----------------------------------------------------------------------------
// BMP loader (24-bit)
// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
// UAV initialization – 5 Rows (Yard Lines) x 3 Cols (Width)
// -----------------------------------------------------------------------------
void initUAVs()
{
    g_uavs.clear();

    // Yard lines converted to meters from the center (50 yard line = 0)
    // 0 yard line = 50 yards away = 45.72m
    // 25 yard line = 25 yards away = 22.86m
    // 50 yard line = 0m
    std::vector<double> yRows = 
    {
        -45.72, // South Goal Line (0 yd line)
        -22.86, // South 25 yd line
        0.0,    // Center 50 yd line
        22.86,  // North 25 yd line
        45.72   // North Goal Line (0 yd line)
    };

    // 3 UAVs across the width per row
    // Spaced out by 15 meters: Left(-15), Center(0), Right(15)
    std::vector<double> xCols = { -15.0, 0.0, 15.0 };

    int idCounter = 0;
    for (double y : yRows)
    {
        for (double x : xCols)
        {
            Vec3 startPos(x, y, 0.0);
            auto uav = std::make_unique<ECE_UAV>(idCounter, startPos, kNumUAVs);
            uav->start();
            g_uavs.push_back(std::move(uav));
            ++idCounter;
        }
    }
}

// -----------------------------------------------------------------------------
// Draw textured field
// -----------------------------------------------------------------------------
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

    // Texture Mapping Rotated 90 degrees so the "Long" side of image 
    // maps to the "Long" side of the geometry (Y-axis).
    
    // Bottom-Left of Geometry (-hw, -hl) -> Map to Left-Top of Image (0, 1)
    // (Assuming image is horizontal landscape)
    
    // Note: Adjust these coords if your specific bitmap is oriented differently.
    // Standard logic: Texture U (0->1) is width, V (0->1) is height.
    // We want Texture Width to run along Field Length (Y).
    
    glTexCoord2f(1.0f, 0.0f); glVertex3f(-hw, -hl, 0.0f); // SW corner
    glTexCoord2f(1.0f, 1.0f); glVertex3f( hw, -hl, 0.0f); // SE corner
    glTexCoord2f(0.0f, 1.0f); glVertex3f( hw,  hl, 0.0f); // NE corner
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-hw,  hl, 0.0f); // NW corner

    glEnd();
    glDisable(GL_TEXTURE_2D);
}

// -----------------------------------------------------------------------------
// Draw virtual wireframe sphere (center (0,0,50), radius 10)
// -----------------------------------------------------------------------------
void drawVirtualSphere()
{
    glDisable(GL_TEXTURE_2D);

    glPushMatrix();
    glTranslatef(0.0f, 0.0f, 50.0f);
    glColor3f(0.7f, 0.7f, 0.9f);
    glutWireSphere(10.0, 24, 24);
    glPopMatrix();
}

// -----------------------------------------------------------------------------
// Display callback
// -----------------------------------------------------------------------------
void displayCallback()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Camera: back and up, looking toward the origin
    gluLookAt(0.0, -110.0, 55.0,
            0.0,    0.0, 20.0,
            0.0,    0.0,  1.0);

    // Rotate world so field length runs horizontally across the screen
    glRotatef(90.0f, 0.0f, 0.0f, 1.0f);

    GLfloat light_pos[] = { 80.0f, -40.0f, 140.0f, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

    drawField();
    drawVirtualSphere();

    // Draw UAVs
    for (const auto& uav : g_uavs)
    {
        Vec3 pos, vel;
        uav->getState(pos, vel);

        glPushMatrix();
        glTranslatef(static_cast<float>(pos.x),
                     static_cast<float>(pos.y),
                     static_cast<float>(pos.z));

        GLfloat mat_col[] = { 0.9f, 0.1f, 0.1f, 1.0f };
        glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_col);

        glutSolidSphere(0.3, 16, 16);
        glPopMatrix();
    }

    glutSwapBuffers();
}


// -----------------------------------------------------------------------------
// Timer: simple collision handling + redraw
// -----------------------------------------------------------------------------
void timerCallback(int)
{
    for (size_t i = 0; i < g_uavs.size(); ++i)
    {
        for (size_t j = i + 1; j < g_uavs.size(); ++j)
        {
            Vec3 p1, v1;
            Vec3 p2, v2;
            g_uavs[i]->getState(p1, v1);
            g_uavs[j]->getState(p2, v2);

            double dist = (p1 - p2).length();
            if (dist < 0.21)   // 20cm box + margin
            {
                g_uavs[i]->setVelocity(v2);
                g_uavs[j]->setVelocity(v1);
            }
        }
    }

    glutPostRedisplay();
    glutTimerFunc(30, timerCallback, 0);
}

// -----------------------------------------------------------------------------
// Reshape
// -----------------------------------------------------------------------------
void reshapeCallback(int width, int height)
{
    if (height == 0) height = 1;
    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0,
                   static_cast<double>(width) /
                   static_cast<double>(height),
                   1.0, 600.0);
    glMatrixMode(GL_MODELVIEW);
}

// -----------------------------------------------------------------------------
// OpenGL init
// -----------------------------------------------------------------------------
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

    // NEW: darker navy-blue background
    glClearColor(0.02f, 0.05f, 0.20f, 1.0f);
}

// -----------------------------------------------------------------------------
// main
// -----------------------------------------------------------------------------
int main(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(kWindowWidth, kWindowHeight);
    glutCreateWindow("ECE 4122 Final Project - UAV Show");

    initOpenGL();

    g_fieldTexture = loadBMPTexture("ff.bmp");
    if (g_fieldTexture != 0)
        g_haveTexture = true;

    initUAVs();

    glutDisplayFunc(displayCallback);
    glutReshapeFunc(reshapeCallback);
    glutTimerFunc(30, timerCallback, 0);

    glutMainLoop();
    return 0;
}