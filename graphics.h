#ifndef graphics_h
#define graphics_h

#include <stdlib.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

/** Default Final Variables */
const int DEFAULT_WINDOW_WIDTH = 1200;
const int DEFAULT_WINDOW_HEIGHT = 800;
const double DEFAULT_RENDER_COLOR_RGB[3] = {0.0, 0.0, 0.0};
const double DEFAULT_BACKGROUND_COLOR_RGB[3] = {0.0, 0.0, 0.0};


// Program initialization NOT OpenGL/GLUT dependent,
// as we haven't created a GLUT window yet
void init();

// Initialize OpenGL Graphics
void initGL();

// Callback functions for GLUT

// Draw the window - this is where all the GL actions are
void display();

// Trap and process alphanumeric keyboard events
void kbd(unsigned char key, int x, int y);

// Trap and process special keyboard events
void kbdS(int key, int x, int y);

// Handle "mouse cursor moved" events
void cursor(int x, int y);

// Calls itself after a specified time
void timer(int dummy);

// Handle mouse button pressed and released events
void mouse(int button, int state, int x, int y);

#endif /* graphics_h */
