#include "graphics.h"
#include "Scene.h"
#include <iostream>
#include <vector>
using namespace std;


GLdouble width, height;
int wd;

Scene scene;

// Used to map 2d coordinates to the screen
//double ORTHO_ZOOM = -1;

/* Movement Keybinds */
const char UP_KEY = ' ';
const char DOWN_KEY = GLUT_ACTIVE_SHIFT;
const char RIGHT_KEY = 'd';
const char LEFT_KEY = 'a';
const char FORWARD_KEY = 'w';
const char BACKWARD_KEY = 's';
const char OUT_KEY = 'q';
const char IN_KEY = 'e';

/* Rotation Keybinds */
const char ROT_RIGHT_KEY = 'l';
const char ROT_LEFT_KEY = 'j';
const char ROT_UP_KEY = 'i';
const char ROT_DOWN_KEY = 'k';
const char ROT_OUT_KEY = 'o';
const char ROT_IN_KEY = 'u';

/* Other Keybinds */
const char TOGGLE_ACTIVE_CAMERA_KEY = 't';


/**
 * Create and Initialize Scene
 */
void init() {
    width = 800;
    height = 500;

    /** Create Axes */
    /// X Axis
    vector<point3d> xAxisPoints({
        point3d(0, 0, 0),
        point3d(10, 0, 0),
        point3d(1, 0, 0),
        point3d(1, 0.1, 0),
        point3d(2, 0, 0),
        point3d(2, 0.1, 0),
        point3d(3, 0, 0),
        point3d(3, 0.1, 0),
        point3d(4, 0, 0),
        point3d(4, 0.1, 0),
        point3d(5, 0, 0),
        point3d(5, 0.1, 0),
        point3d(6, 0, 0),
        point3d(6, 0.1, 0),
        point3d(7, 0, 0),
        point3d(7, 0.1, 0),
        point3d(8, 0, 0),
        point3d(8, 0.1, 0),
        point3d(9, 0, 0),
        point3d(9, 0.1, 0),
        point3d(10, 0, 0),
        point3d(10, 0.1, 0),
    });
    vector<edge3d> xAxisEdges({
        edge3d(xAxisPoints[0], xAxisPoints[1]),
        edge3d(xAxisPoints[2], xAxisPoints[3]),
        edge3d(xAxisPoints[4], xAxisPoints[5]),
        edge3d(xAxisPoints[6], xAxisPoints[7]),
        edge3d(xAxisPoints[8], xAxisPoints[9]),
        edge3d(xAxisPoints[10], xAxisPoints[11]),
        edge3d(xAxisPoints[12], xAxisPoints[13]),
        edge3d(xAxisPoints[14], xAxisPoints[15]),
        edge3d(xAxisPoints[16], xAxisPoints[17]),
        edge3d(xAxisPoints[18], xAxisPoints[19]),
        edge3d(xAxisPoints[20], xAxisPoints[21]),
    });
    scene.addObject(Object3D(xAxisPoints, xAxisEdges));

    /// Y Axis
    vector<point3d> yAxisPoints({
        point3d(0, 0, 0),
        point3d(0, 10, 0),
        point3d(0, 1, 0),
        point3d(0.1, 1, 0),
        point3d(0, 2, 0),
        point3d(0.1, 2, 0),
        point3d(0, 3, 0),
        point3d(0.1, 3, 0),
        point3d(0, 4, 0),
        point3d(0.1, 4, 0),
        point3d(0, 5, 0),
        point3d(0.1, 5, 0),
        point3d(0, 6, 0),
        point3d(0.1, 6, 0),
        point3d(0, 7, 0),
        point3d(0.1, 7, 0),
        point3d(0, 8, 0),
        point3d(0.1, 8, 0),
        point3d(0, 9, 0),
        point3d(0.1, 9, 0),
        point3d(0, 10, 0),
        point3d(0.1, 10, 0),
    });
    vector<edge3d> yAxisEdges({
        edge3d(yAxisPoints[0], yAxisPoints[1]),
        edge3d(yAxisPoints[2], yAxisPoints[3]),
        edge3d(yAxisPoints[4], yAxisPoints[5]),
        edge3d(yAxisPoints[6], yAxisPoints[7]),
        edge3d(yAxisPoints[8], yAxisPoints[9]),
        edge3d(yAxisPoints[10], yAxisPoints[11]),
        edge3d(yAxisPoints[12], yAxisPoints[13]),
        edge3d(yAxisPoints[14], yAxisPoints[15]),
        edge3d(yAxisPoints[16], yAxisPoints[17]),
        edge3d(yAxisPoints[18], yAxisPoints[19]),
        edge3d(yAxisPoints[20], yAxisPoints[21]),
    });
    scene.addObject(Object3D(yAxisPoints, yAxisEdges));

    /// Z Axis
    vector<point3d> zAxisPoints({
        point3d(0, 0, 0),
        point3d(0, 0, 10),
        point3d(0, 0, 1),
        point3d(0.1, 0, 1),
        point3d(0, 0, 2),
        point3d(0.1, 0, 2),
        point3d(0, 0, 3),
        point3d(0.1, 0, 3),
        point3d(0, 0, 4),
        point3d(0.1, 0, 4),
        point3d(0, 0, 5),
        point3d(0.1, 0, 5),
        point3d(0, 0, 6),
        point3d(0.1, 0, 6),
        point3d(0, 0, 7),
        point3d(0.1, 0, 7),
        point3d(0, 0, 8),
        point3d(0.1, 0, 8),
        point3d(0, 0, 9),
        point3d(0.1, 0, 9),
        point3d(0, 0, 10),
        point3d(0.1, 0, 10),
    });
    vector<edge3d> zAxisEdges({
        edge3d(zAxisPoints[0], zAxisPoints[1]),
        edge3d(zAxisPoints[2], zAxisPoints[3]),
        edge3d(zAxisPoints[4], zAxisPoints[5]),
        edge3d(zAxisPoints[6], zAxisPoints[7]),
        edge3d(zAxisPoints[8], zAxisPoints[9]),
        edge3d(zAxisPoints[10], zAxisPoints[11]),
        edge3d(zAxisPoints[12], zAxisPoints[13]),
        edge3d(zAxisPoints[14], zAxisPoints[15]),
        edge3d(zAxisPoints[16], zAxisPoints[17]),
        edge3d(zAxisPoints[18], zAxisPoints[19]),
        edge3d(zAxisPoints[20], zAxisPoints[21]),
    });
    scene.addObject(Object3D(zAxisPoints, zAxisEdges));

    /// Create 3d Cube
    double CUBE_SIZE = 1;

    vector<point3d> cube3dPoints({
        point3d(1 * CUBE_SIZE, 1 * CUBE_SIZE, 1 * CUBE_SIZE),
        point3d(1 * CUBE_SIZE, 1 * CUBE_SIZE, -1 * CUBE_SIZE),
        point3d(1 * CUBE_SIZE, -1 * CUBE_SIZE, 1 * CUBE_SIZE),
        point3d(1 * CUBE_SIZE, -1 * CUBE_SIZE, -1 * CUBE_SIZE),
        point3d(-1 * CUBE_SIZE, 1 * CUBE_SIZE, 1 * CUBE_SIZE),
        point3d(-1 * CUBE_SIZE, 1 * CUBE_SIZE, -1 * CUBE_SIZE),
        point3d(-1 * CUBE_SIZE, -1 * CUBE_SIZE, 1 * CUBE_SIZE),
        point3d(-1 * CUBE_SIZE, -1 * CUBE_SIZE, -1 * CUBE_SIZE)
    });
    vector<edge3d> cube3dEdges({
        edge3d(cube3dPoints[0], cube3dPoints[1]),
        edge3d(cube3dPoints[0], cube3dPoints[2]),
        edge3d(cube3dPoints[0], cube3dPoints[4]),
        edge3d(cube3dPoints[1], cube3dPoints[3]),
        edge3d(cube3dPoints[1], cube3dPoints[5]),
        edge3d(cube3dPoints[2], cube3dPoints[3]),
        edge3d(cube3dPoints[2], cube3dPoints[6]),
        edge3d(cube3dPoints[3], cube3dPoints[7]),
        edge3d(cube3dPoints[4], cube3dPoints[5]),
        edge3d(cube3dPoints[4], cube3dPoints[6]),
        edge3d(cube3dPoints[5], cube3dPoints[7]),
        edge3d(cube3dPoints[6], cube3dPoints[7])
    });
    scene.addObject(Object3D(cube3dPoints, cube3dEdges));

    /// Create 4d Cube
    vector<point4d> cube4dPoints({
        point4d(1 * CUBE_SIZE, 5 * CUBE_SIZE, 1 * CUBE_SIZE, 1),
        point4d(1 * CUBE_SIZE, 5 * CUBE_SIZE, -1 * CUBE_SIZE, 1),
        point4d(1 * CUBE_SIZE, 7 * CUBE_SIZE, 1 * CUBE_SIZE, 1),
        point4d(1 * CUBE_SIZE, 7 * CUBE_SIZE, -1 * CUBE_SIZE, 1),
        point4d(-1 * CUBE_SIZE, 5 * CUBE_SIZE, 1 * CUBE_SIZE, 1),
        point4d(-1 * CUBE_SIZE, 5 * CUBE_SIZE, -1 * CUBE_SIZE, 1),
        point4d(-1 * CUBE_SIZE, 7 * CUBE_SIZE, 1 * CUBE_SIZE, 1),
        point4d(-1 * CUBE_SIZE, 7 * CUBE_SIZE, -1 * CUBE_SIZE, 1),
        point4d(1 * CUBE_SIZE, 5 * CUBE_SIZE, 1 * CUBE_SIZE, -1),
        point4d(1 * CUBE_SIZE, 5 * CUBE_SIZE, -1 * CUBE_SIZE, -1),
        point4d(1 * CUBE_SIZE, 7 * CUBE_SIZE, 1 * CUBE_SIZE, -1),
        point4d(1 * CUBE_SIZE, 7 * CUBE_SIZE, -1 * CUBE_SIZE, -1),
        point4d(-1 * CUBE_SIZE, 5 * CUBE_SIZE, 1 * CUBE_SIZE, -1),
        point4d(-1 * CUBE_SIZE, 5 * CUBE_SIZE, -1 * CUBE_SIZE, -1),
        point4d(-1 * CUBE_SIZE, 7 * CUBE_SIZE, 1 * CUBE_SIZE, -1),
        point4d(-1 * CUBE_SIZE, 7 * CUBE_SIZE, -1 * CUBE_SIZE, -1),
    });
    vector<edge4d> cube4dEdges({
        edge4d(cube4dPoints[0], cube4dPoints[1]),
        edge4d(cube4dPoints[0], cube4dPoints[2]),
        edge4d(cube4dPoints[0], cube4dPoints[4]),
        edge4d(cube4dPoints[0], cube4dPoints[8]),
        edge4d(cube4dPoints[1], cube4dPoints[3]),
        edge4d(cube4dPoints[1], cube4dPoints[5]),
        edge4d(cube4dPoints[1], cube4dPoints[9]),
        edge4d(cube4dPoints[2], cube4dPoints[3]),
        edge4d(cube4dPoints[2], cube4dPoints[10]),
        edge4d(cube4dPoints[2], cube4dPoints[6]),
        edge4d(cube4dPoints[3], cube4dPoints[7]),
        edge4d(cube4dPoints[3], cube4dPoints[11]),
        edge4d(cube4dPoints[4], cube4dPoints[5]),
        edge4d(cube4dPoints[4], cube4dPoints[6]),
        edge4d(cube4dPoints[4], cube4dPoints[12]),
        edge4d(cube4dPoints[5], cube4dPoints[7]),
        edge4d(cube4dPoints[5], cube4dPoints[13]),
        edge4d(cube4dPoints[6], cube4dPoints[7]),
        edge4d(cube4dPoints[6], cube4dPoints[14]),
        edge4d(cube4dPoints[7], cube4dPoints[15]),
        edge4d(cube4dPoints[8], cube4dPoints[9]),
        edge4d(cube4dPoints[8], cube4dPoints[10]),
        edge4d(cube4dPoints[8], cube4dPoints[12]),
        edge4d(cube4dPoints[9], cube4dPoints[11]),
        edge4d(cube4dPoints[9], cube4dPoints[13]),
        edge4d(cube4dPoints[10], cube4dPoints[11]),
        edge4d(cube4dPoints[10], cube4dPoints[14]),
        edge4d(cube4dPoints[11], cube4dPoints[15]),
        edge4d(cube4dPoints[12], cube4dPoints[13]),
        edge4d(cube4dPoints[12], cube4dPoints[14]),
        edge4d(cube4dPoints[13], cube4dPoints[15]),
        edge4d(cube4dPoints[14], cube4dPoints[15]),
    });
    scene.addObject(Object4D(cube4dPoints, cube4dEdges));
}

/* Initialize OpenGL Graphics */
void initGL() {
    // Set "clearing" or background color 0, 113, 85, 1
    glClearColor(0.9f, 0.9f, 0.86f, 1.0f); // Black and opaque
}

/* Handler for window-repaint event. Call back when the window first appears and
 whenever the window needs to be re-painted. */
void display() {
    // Tell OpenGL to use the whole window for drawing
    glViewport(0, 0, width, height);
//    glViewport(0, 0, width, height);

    // Do an orthographic parallel projection with the coordinate
    // system set to center, limited by screen/window size
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
//    glOrtho(-ORTHO_ZOOM/width/2, ORTHO_ZOOM/width/2, -ORTHO_ZOOM/height/2,
//        ORTHO_ZOOM/height/2, 1.f, -1.f);
    glOrtho(-width/2, width/2, -height/2, height/2, 1.f, -1.f);

    // Clear the color buffer with current clearing color
    glClear(GL_COLOR_BUFFER_BIT);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    scene.draw();

    // Render now
    glFlush();
}

// http://www.theasciicode.com.ar/ascii-control-characters/escape-ascii-code-27.html
void kbd(unsigned char key, int x, int y)
{
    // Get state of shift/ctrl/alt etc.
    glutGetModifiers();

    switch(key) {
        case 27:
            // escape
            glutDestroyWindow(wd);
            exit(0);
            break;
        case ROT_UP_KEY:
            if (scene.getActiveCamera()){
                scene.getCamera3D().rotateUp();
            } else {
                scene.getCamera4D().rotateUp();
            }
            break;
        case ROT_DOWN_KEY:
            if (scene.getActiveCamera()){
                scene.getCamera3D().rotateDown();
            } else {
                scene.getCamera4D().rotateDown();
            }
            break;
        case ROT_RIGHT_KEY:
            if (scene.getActiveCamera()){
                scene.getCamera3D().rotateRight();
            } else {
                scene.getCamera4D().rotateRight();
            }
            break;
        case ROT_LEFT_KEY:
            if (scene.getActiveCamera()){
                scene.getCamera3D().rotateLeft();
            } else {
                scene.getCamera4D().rotateLeft();
            }
            break;
        case ROT_OUT_KEY:
            // Only can rotate out if 4d camera is selected
            if (!scene.getActiveCamera()){
                scene.getCamera4D().rotateOut();
            }
            break;
        case ROT_IN_KEY:
            // Only can rotate in if 4d camera is selected
            if (!scene.getActiveCamera()){
                scene.getCamera4D().rotateIn();
            }
            break;
        case UP_KEY:
            if (scene.getActiveCamera()){
                scene.getCamera3D().up();
            } else {
                scene.getCamera4D().up();
            }
            break;
        case DOWN_KEY:
            if (scene.getActiveCamera()){
                scene.getCamera3D().down();
            } else {
                scene.getCamera4D().down();
            }
            break;
        case RIGHT_KEY:
            if (scene.getActiveCamera()){
                scene.getCamera3D().right();
            } else {
                scene.getCamera4D().right();
            }
            break;
        case LEFT_KEY:
            if (scene.getActiveCamera()){
                scene.getCamera3D().left();
            } else {
                scene.getCamera4D().left();
            }
            break;
        case FORWARD_KEY:
            if (scene.getActiveCamera()){
                scene.getCamera3D().forward();
            } else {
                scene.getCamera4D().forward();
            }
            break;
        case BACKWARD_KEY:
            if (scene.getActiveCamera()){
                scene.getCamera3D().back();
            } else {
                scene.getCamera4D().back();
            }
            break;
        case OUT_KEY:
            // Only can move outward if 4d camera is selected
            if (!scene.getActiveCamera()){
                scene.getCamera4D().out();
            }
            break;
        case IN_KEY:
            // Only can move outward if 4d camera is selected
            if (!scene.getActiveCamera()){
                scene.getCamera4D().in();
            }
            break;
        case TOGGLE_ACTIVE_CAMERA_KEY:
            scene.toggleActiveCamera();
            break;
    }

    glutPostRedisplay();
}

void kbdS(int key, int x, int y) {
    glutGetModifiers();
    if (GLUT_ACTIVE_SHIFT) { // DOWN_KEY
        if (scene.getActiveCamera()){
            scene.getCamera3D().down();
        } else {
            scene.getCamera4D().down();
        }
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
//    /** Create and Initialize Scene */
//    vector<point3d> cube3dPoints({
//        point3d(1, 5, 1),
//        point3d(1, 5, -1),
//        point3d(1, 7, 1),
//        point3d(1, 7, -1),
//        point3d(-1, 5, 1),
//        point3d(-1, 5, -1),
//        point3d(-1, 7, 1),
//        point3d(-1, 7, -1),
//    });
//    vector<edge3d> cube3dEdges({
//        edge3d(cube3dPoints[0], cube3dPoints[1]),
//        edge3d(cube3dPoints[0], cube3dPoints[2]),
//        edge3d(cube3dPoints[0], cube3dPoints[4]),
//        edge3d(cube3dPoints[1], cube3dPoints[3]),
//        edge3d(cube3dPoints[1], cube3dPoints[5]),
//        edge3d(cube3dPoints[2], cube3dPoints[3]),
//        edge3d(cube3dPoints[2], cube3dPoints[6]),
//        edge3d(cube3dPoints[3], cube3dPoints[7]),
//        edge3d(cube3dPoints[4], cube3dPoints[5]),
//        edge3d(cube3dPoints[4], cube3dPoints[6]),
//        edge3d(cube3dPoints[5], cube3dPoints[7]),
//        edge3d(cube3dPoints[6], cube3dPoints[7]),
//
//    });
//
//    Object3D testCube(cube3dPoints, cube3dEdges);
//
//    // Extrude single edge twice into a cube
////    testCube.extrude(spatialVector(std::vector<double>({
////        0, 2, 0
////    })));
////    testCube.extrude(spatialVector(std::vector<double>({
////        0, 0, 2
////    })));
//    scene.addObject(testCube);


    /** GLUT Processes */
    init();

    // Initialize GLUT
    glutInit(&argc, argv);

    glutInitDisplayMode(GLUT_RGBA);

    glutInitWindowSize((int)width, (int)height);

    // Position the window's initial top-left corner
    glutInitWindowPosition( 100, 100);

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
