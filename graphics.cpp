#include "graphics.h"
#include <iostream>
#include <vector>
using namespace std;

GLdouble width, height;
int wd;

/* Shape Variables */
// Square
const double SQUARE_START_LOC[] {60, 190};
const double SQUARE_START_SCALE = 50;
int squareCurrentLoc[] {};
int squareCurrentScale;
int squarePointLocationsRelative[5][2] {
    {0, 0},
    {0, 1},
    {1, 1},
    {1, 0}
};

// Rhombus
const int RHOMBUS_START_LOC[] {};


void init() {
    width = 600;
    height = 400;
}

/* Initialize OpenGL Graphics */
void initGL() {
    // Set "clearing" or background color 0, 113, 85, 1
    glClearColor(0.9f, 0.9f, 0.8f, 1.0f); // Black and opaque
}

/* Handler for window-repaint event. Call back when the window first appears and
 whenever the window needs to be re-painted. */
void display() {
    // Tell OpenGL to use the whole window for drawing
    glViewport(0, 0, width, height);

    // Do an orthographic parallel projection with the coordinate
    // system set to center, limited by screen/window size
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-width/2, width/2, -height/2, height/2, 1.f, -1.f);

    // Clear the color buffer with current clearing color
    glClear(GL_COLOR_BUFFER_BIT);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // Render now
    glFlush();
}

// http://www.theasciicode.com.ar/ascii-control-characters/escape-ascii-code-27.html
void kbd(unsigned char key, int x, int y)
{
    // escape
    if (key == 27) {
        glutDestroyWindow(wd);
        exit(0);
    }

    glutPostRedisplay();
}

void kbdS(int key, int x, int y) {
    switch(key) {
        case GLUT_KEY_DOWN:

            break;
        case GLUT_KEY_LEFT:

            break;
        case GLUT_KEY_RIGHT:

            break;
        case GLUT_KEY_UP:

            break;
    }

    glutPostRedisplay();
}

void cursor(int x, int y) {

    glutPostRedisplay();
}

// button will be GLUT_LEFT_BUTTON or GLUT_RIGHT_BUTTON
// state will be GLUT_UP or GLUT_DOWN
void mouse(int button, int state, int x, int y) {

    glutPostRedisplay();
}

void timer(int dummy) {

    glutPostRedisplay();
    glutTimerFunc(30, timer, dummy);
}

/* Main function: GLUT runs as a console application starting at main()  */
int main(int argc, char** argv) {
    init();

    glutInit(&argc, argv);          // Initialize GLUT

    glutInitDisplayMode(GLUT_RGBA);

    glutInitWindowSize((int)width, (int)height);
    glutInitWindowPosition(-2500, 100); // Position the window's initial
    // top-left
    // corner
    /* create the window and store the handle to it */
    wd = glutCreateWindow(/* Fun with Drawing! */ "/* title */");

    // Register callback handler for window re-paint event
    glutDisplayFunc(display);

    // Our own OpenGL initialization
    initGL();

    // register keyboard press event processing function
    // works for numbers, letters, spacebar, etc.
    glutKeyboardFunc(kbd);

    // register special event: function keys, arrows, etc.
    glutSpecialFunc(kbdS);

    // handles mouse movement
    glutPassiveMotionFunc(cursor);

    // handles mouse click
    glutMouseFunc(mouse);

    // handles timer
    glutTimerFunc(0, timer, 0);

    // Enter the event-processing loop
    glutMainLoop();

    return 0;
}
