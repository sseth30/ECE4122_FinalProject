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

// ---------------------------------------------------------------------------
// Global constants for window and field geometry
// ---------------------------------------------------------------------------
static const int   kWindowWidth  = 1000;
static const int   kWindowHeight = 750;

// Field dimensions in meters. Length is along y, width is along x.
static const float kFieldLength  = 118.44f; 
static const float kFieldWidth   = 48.76f; 

static const int   kNumUAVs      = 15;

// ---------------------------------------------------------------------------
// Global simulation state
// ---------------------------------------------------------------------------
static std::vector<std::unique_ptr<ECE_UAV>> g_uavs;
static Mesh g_uavMesh;
static bool g_haveMesh = false;

static GLuint g_fieldTexture = 0;
static bool   g_haveTexture  = false;

static GLuint g_uavTexture   = 0;
static bool   g_haveUavTex   = false;


// Forward declarations of GLUT callbacks
void displayCallback();
void reshapeCallback(int width, int height);
void timerCallback(int value);

/*
 * Function: loadBMPTexture
 * Purpose : Load a 24-bit BMP file from disk into an OpenGL 2D texture.
 * Input   : filename - C-style string giving the path to the BMP file.
 * Output  : None (texture data is uploaded into OpenGL).
 * Return  : GLuint texture id created by OpenGL, or 0 if loading fails.
 */
GLuint loadBMPTexture(const char* filename)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file)
    {
        std::cerr << "Error: Could not open texture file "
                  << filename << std::endl;
        return 0;
    }

    // Read the 54 byte BMP header.
    unsigned char header[54];
    if (!file.read(reinterpret_cast<char*>(header), 54))
        return 0;

    // Very simple validity check: first two characters must be "BM".
    if (header[0] != 'B' || header[1] != 'M')
        return 0;

    // Extract header fields we care about.
    unsigned int dataPos   = *reinterpret_cast<unsigned int*>(&header[0x0A]);
    unsigned int imageSize = *reinterpret_cast<unsigned int*>(&header[0x22]);
    unsigned int width     = *reinterpret_cast<unsigned int*>(&header[0x12]);
    unsigned int height    = *reinterpret_cast<unsigned int*>(&header[0x16]);

    // Some BMP writers leave these as zero, so compute defaults if needed.
    if (imageSize == 0)
        imageSize = width * height * 3;
    if (dataPos == 0)
        dataPos = 54;

    // Read raw BGR pixel data.
    std::vector<unsigned char> data(imageSize);
    file.seekg(dataPos);
    file.read(reinterpret_cast<char*>(data.data()), imageSize);
    file.close();

    // Create and configure an OpenGL texture.
    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    // Upload the BGR data as an RGB texture.
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                 width, height, 0,
                 GL_BGR, GL_UNSIGNED_BYTE,
                 data.data());

    return textureID;
}

/*
 * Function: initUAVs
 * Purpose : Create the 15 ECE_UAV objects and place them on the field
 *           at the 0, 25, 50, 25, 0 yard lines as shown in the project spec.
 * Input   : None.
 * Output  : Populates the global vector g_uavs and starts each UAV thread.
 * Return  : None.
 */
void initUAVs()
{
    g_uavs.clear();

    // Yard lines (in meters) relative to field center at the 50 yard line.
    std::vector<double> yRows = 
    {
        -45.72, 
        -22.86,
        0.0,   
        22.86,  
        45.72   
    };

    // Three UAVs across the width for each yard line.
    std::vector<double> xCols = { -15.0, 0.0, 15.0 };

    int idCounter = 0;
    for (double y : yRows)
    {
        for (double x : xCols)
        {
            // Start all UAVs at ground level (z = 0).
            Vec3 startPos(x, y, 0.0);
            auto uav = std::make_unique<ECE_UAV>(idCounter, startPos, kNumUAVs);
            uav->start();                  // Launch control thread for this UAV.
            g_uavs.push_back(std::move(uav));
            ++idCounter;
        }
    }
}

/*
 * Function: drawField
 * Purpose : Render the football field as a textured rectangle centered
 *           at the origin on the z = 0 plane.
 * Input   : None.
 * Output  : Issues OpenGL draw calls to render the field.
 * Return  : None.
 */
void drawField()
{
    // Basic material so lighting interacts with the field.
    GLfloat mat_ambient[]  = { 0.4f, 0.4f, 0.4f, 1.0f };
    GLfloat mat_diffuse[]  = { 0.9f, 0.9f, 0.9f, 1.0f };
    GLfloat mat_specular[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    glMaterialfv(GL_FRONT, GL_AMBIENT,  mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE,  mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);

    // Bind the field texture if it loaded correctly.
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

    // Draw a single textured quad for the field.
    glBegin(GL_QUADS);
    glNormal3f(0.0f, 0.0f, 1.0f);
    
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-hw, -hl, 0.0f); // SW
    glTexCoord2f(0.0f, 1.0f); glVertex3f( hw, -hl, 0.0f); // SE
    glTexCoord2f(1.0f, 1.0f); glVertex3f( hw,  hl, 0.0f); // NE
    glTexCoord2f(1.0f, 0.0f); glVertex3f(-hw,  hl, 0.0f); // NW

    glEnd();
    glDisable(GL_TEXTURE_2D);
}

/*
 * Function: displayCallback
 * Purpose : GLUT display callback. Sets up the camera, positions the light,
 *           draws the field, and draws all UAVs at their current locations.
 * Input   : None (GLUT callback signature has no explicit parameters).
 * Output  : Renders one frame to the OpenGL back buffer.
 * Return  : None.
 */
void displayCallback()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Camera placed above and behind one corner, looking toward origin.
    gluLookAt(100.0, -60.0, 100.0,  
              0.0,    0.0,   0.0,  
              0.0,    0.0,   1.0); 

    // Simple single positional light.
    GLfloat light_pos[] = { 80.0f, -40.0f, 140.0f, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

    // Draw the football field.
    drawField();

    // Draw each UAV using its current simulated position.
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
            // Optionally enable the metal texture for the UAV mesh.
            if (g_haveUavTex)
            {
                glEnable(GL_TEXTURE_2D);
                glBindTexture(GL_TEXTURE_2D, g_uavTexture);
            }

            GLfloat mat_diff[]    = { 1.0f, 1.0f, 1.0f, 1.0f };
            GLfloat mat_spec[]    = { 0.3f, 0.3f, 0.3f, 1.0f };
            GLfloat mat_shininess[] = { 20.0f };
            glMaterialfv(GL_FRONT, GL_DIFFUSE,  mat_diff);
            glMaterialfv(GL_FRONT, GL_SPECULAR, mat_spec);
            glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);

            // Draw the loaded OBJ mesh representing the UAV.
            g_uavMesh.draw();

            if (g_haveUavTex)
                glDisable(GL_TEXTURE_2D);
        }
        else
        {
            // Fallback: draw a simple red sphere if no mesh is loaded.
            GLfloat mat_col[] = { 1.0f, 0.0f, 0.0f, 1.0f };
            glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_col);
            glutSolidSphere(1.0, 16, 16); 
        }

        glPopMatrix();
    }

    glutSwapBuffers();
}

/*
 * Function: timerCallback
 * Purpose : GLUT timer callback that performs simple collision handling
 *           between UAVs and triggers a redraw at regular intervals.
 * Input   : value - integer passed by GLUT (unused here).
 * Output  : Updates UAV velocities when they are too close and schedules
 *           the next timer event and redraw.
 * Return  : None.
 */
void timerCallback(int)
{
    // Check all unordered pairs of UAVs for potential collisions.
    for (size_t i = 0; i < g_uavs.size(); ++i)
    {
        for (size_t j = i + 1; j < g_uavs.size(); ++j)
        {
            Vec3 p1, v1;
            Vec3 p2, v2;
            g_uavs[i]->getState(p1, v1);
            g_uavs[j]->getState(p2, v2);

            double dist = (p1 - p2).length();
            if (dist < 0.21)   
            {
                // Simple elastic collision model: swap velocity vectors.
                g_uavs[i]->setVelocity(v2);
                g_uavs[j]->setVelocity(v1);
            }
        }
    }

    // Request a redraw and reschedule the timer.
    glutPostRedisplay();
    glutTimerFunc(30, timerCallback, 0);
}

/*
 * Function: reshapeCallback
 * Purpose : GLUT reshape callback. Updates the viewport and projection
 *           matrix when the window size changes.
 * Input   : width  - new window width in pixels.
 *           height - new window height in pixels.
 * Output  : Sets OpenGL viewport and projection matrix.
 * Return  : None.
 */
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

/*
 * Function: initOpenGL
 * Purpose : One-time OpenGL initialization. Enables depth testing,
 *           lighting, and sets global lighting and clear color.
 * Input   : None.
 * Output  : Configures OpenGL state machine for the simulation.
 * Return  : None.
 */
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

    // Dark navy blue background for the night sky.
    glClearColor(0.02f, 0.05f, 0.20f, 1.0f);
}

/*
 * Function: main
 * Purpose : Program entry point. Sets up GLUT, OpenGL state, loads textures
 *           and OBJ mesh, initializes the UAVs, and hands control to GLUT.
 * Input   : argc - number of command line arguments.
 *           argv - array of C-style strings for each argument. Optionally
 *                  argv[1] can override the default OBJ file name.
 * Output  : Creates the window and starts the GLUT event loop.
 * Return  : int status code for the operating system (0 on normal exit).
 */
int main(int argc, char** argv)
{
    std::string objPath = "Torus.obj";
    if (argc >= 2) objPath = argv[1];

    // Initialize GLUT and create the rendering window.
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(kWindowWidth, kWindowHeight);
    glutCreateWindow("ECE 4122 Final Project - UAV Show");

    // Set up core OpenGL state (lighting, depth test, clear color).
    initOpenGL();

    // Load the UAV mesh and scale it so its bounding box is about 1.5 meters.
    g_haveMesh = g_uavMesh.loadFromOBJ(objPath, 1.5);

    // Load the football field texture (ff.bmp) and mark if it is valid.
    g_fieldTexture = loadBMPTexture("ff.bmp");
    if (g_fieldTexture != 0)
        g_haveTexture = true;
    
    // Load the metal texture for the UAV mesh (metal.bmp).
    g_uavTexture = loadBMPTexture("metal.bmp");
    if (g_uavTexture != 0)
        g_haveUavTex = true;

    // Create and start all UAVs in their initial field positions.
    initUAVs();

    // Register GLUT callbacks.
    glutDisplayFunc(displayCallback);
    glutReshapeFunc(reshapeCallback);
    glutTimerFunc(30, timerCallback, 0);

    // Enter the GLUT main loop. This will not return under normal operation.
    glutMainLoop();
    return 0;
}
