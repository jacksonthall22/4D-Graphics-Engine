#include "graphics.h"
#include "Scene.h"
#include <iostream>
#include <vector>
#include <sstream>
//#include <map>

using namespace std;


GLdouble windowWidth, windowHeight;
int window;


Scene scene;


/**
 * Create and Initialize Scene
 */
void init(){
    windowWidth = DEFAULT_WINDOW_WIDTH;
    windowHeight = DEFAULT_WINDOW_HEIGHT;
    /** Create Axes **/
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
void initGL(){
    // Set "clearing" or background color 0, 113, 85, 1
    glClearColor(0.9f, 0.9f, 0.86f, 1.0f); // Black and opaque
}

/* Handler for window-repaint event. Call back when the window first appears and
 whenever the window needs to be re-painted. */
void display(){
    // Tell OpenGL to use the whole window for drawing
    glViewport(0, 0, windowWidth, windowHeight);

    // Do an orthographic parallel projection with the coordinate
    // system set to center, limited by screen/window size
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glOrtho(-windowWidth/2.0, windowWidth/2.0,
        -windowHeight/2.0, windowHeight/2.0,
        1.f, -1.f);

    // Clear the color buffer with current clearing color
    glClear(GL_COLOR_BUFFER_BIT);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    scene.draw();

    // Render now
    glFlush();
}

void kbd(unsigned char key, int x, int y){
    // Handle all non-movement keys (rotation (for now), toggling camera, etc.)
    switch(tolower(key)){
        case 27:
            // Escape
            glutDestroyWindow(window);
            exit(0);
        case TOGGLE_ACTIVE_CAMERA_KEY:
            scene.toggleActiveCamera();
            break;
    }

    glutPostRedisplay();
}

void kbdS(int key, int x, int y){
//    glutPostRedisplay();
}

void cursor(int x, int y){
//    glutPostRedisplay();
}

// button will be GLUT_LEFT_BUTTON or GLUT_RIGHT_BUTTON
// state will be GLUT_UP or GLUT_DOWN
void mouse(int button, int state, int x, int y){
//    glutPostRedisplay();
}

bool debug = false;
void timer(int dummy){
    /// Get states of all keybinds
    int upPressed,
            downPressed,
            rightPressed,
            leftPressed,
            forwardPressed,
            backPressed,
            outPressed,
            inPressed,
            rotRightPressed,
            rotLeftPressed,
            rotUpPressed,
            rotDownPressed,
            rotOutPressed,
            rotInPressed,
            toggleActiveCameraPressed,
            toggleMovementModePressed;

    /* Movement */
    upPressed = GetAsyncKeyState(UP_KEY);
    downPressed = GetAsyncKeyState(DOWN_KEY);
    rightPressed = GetAsyncKeyState(RIGHT_KEY);
    leftPressed = GetAsyncKeyState(LEFT_KEY);
    forwardPressed = GetAsyncKeyState(FORWARD_KEY);
    backPressed = GetAsyncKeyState(BACK_KEY);
    outPressed = GetAsyncKeyState(OUT_KEY);
    inPressed = GetAsyncKeyState(IN_KEY);

    /* Rotation */
    rotRightPressed = GetAsyncKeyState(ROT_RIGHT_KEY);
    rotLeftPressed = GetAsyncKeyState(ROT_LEFT_KEY);
    rotUpPressed = GetAsyncKeyState(ROT_UP_KEY);
    rotDownPressed = GetAsyncKeyState(ROT_DOWN_KEY);
    rotOutPressed = GetAsyncKeyState(ROT_OUT_KEY);
    rotInPressed = GetAsyncKeyState(ROT_IN_KEY);

    /* Other */
    toggleActiveCameraPressed = GetAsyncKeyState(TOGGLE_ACTIVE_CAMERA_KEY);
    toggleMovementModePressed = GetAsyncKeyState(TOGGLE_MOVEMENT_MODE);

    /** Movement & Rotation **/
    if (scene.getActiveCamera() == Camera::CameraType::Camera3D){
        /** Movement **/
        if (scene.getCamera3D().getMovementMode()
                == Camera::MovementMode::Fixed){
            /// Move 3d in fixed mode

            // Right/left
            if (rightPressed and leftPressed){
                ;
            } else if (rightPressed){
                scene.getCamera3D().moveRelativeDefaultR();
            } else if (leftPressed){
                scene.getCamera3D().moveRelativeDefaultL();
            }

            // Up/down
            if (upPressed and downPressed){
                ;
            } else if (upPressed){
                scene.getCamera3D().moveRelativeDefaultU();
            } else if (downPressed){
                scene.getCamera3D().moveRelativeDefaultD();
            }

            // Forward/back
            if (forwardPressed and backPressed){
                ;
            } else if (forwardPressed){
                scene.getCamera3D().moveRelativeDefaultF();
            } else if (backPressed){
                scene.getCamera3D().moveRelativeDefaultB();
            }

        } else if (scene.getCamera3D().getMovementMode()
                == Camera::MovementMode::Fly){
            /// Move 3d in fly mode

            // Forward/back
            if (forwardPressed and backPressed){
                scene.getCamera3D().applyBrakeFB();
            } else if (forwardPressed){
                scene.getCamera3D().setAccelerationDefaultF();
            } else if (backPressed){
                scene.getCamera3D().setAccelerationDefaultB();
            } else {
                scene.getCamera3D().setAccelerationFB(0);
            }

            // Right/left
            if (rightPressed and leftPressed){
                scene.getCamera3D().applyBrakeRL();
            } else if (rightPressed){
                scene.getCamera3D().setAccelerationDefaultR();
            } else if (leftPressed){
                scene.getCamera3D().setAccelerationDefaultL();
            } else {
                scene.getCamera3D().setAccelerationRL(0);
            }

            // Up/down
            if (upPressed and downPressed){
                scene.getCamera3D().applyBrakeUD();
            } else if (upPressed){
                scene.getCamera3D().setAccelerationDefaultU();
            } else if (downPressed){
                scene.getCamera3D().setAccelerationDefaultD();
            } else {
                scene.getCamera3D().setAccelerationUD(0);
            }

            scene.getCamera3D().moveFly();
        }

        /** Rotation **/
        // Right/left
        if (rotRightPressed and rotLeftPressed){
            ;
        } else if (rotRightPressed){
            scene.getCamera3D().rotateRight();
        } else if (rotLeftPressed){
            scene.getCamera3D().rotateLeft();
        }

        // Up/down
        if (rotUpPressed and rotDownPressed){
            ;
        } else if (rotUpPressed){
            scene.getCamera3D().rotateUp();
        } else if (rotDownPressed){
            scene.getCamera3D().rotateDown();
        }

    } else if (scene.getActiveCamera() == Camera::CameraType::Camera4D){
        /** Movement **/
        // Move scene.camera4d
        if (scene.getCamera4D().getMovementMode()
                == Camera::MovementMode::Fixed){
            /// Move 4d in fixed mode

            // Forward/back
            if (forwardPressed and backPressed){
                ;
            } else if (forwardPressed){
                scene.getCamera4D().moveRelativeDefaultF();
            } else if (backPressed){
                scene.getCamera4D().moveRelativeDefaultB();
            }

            // Right/left
            if (rightPressed and leftPressed){
                ;
            } else if (rightPressed){
                scene.getCamera4D().moveRelativeDefaultR();
            } else if (leftPressed){
                scene.getCamera4D().moveRelativeDefaultL();
            }

            // Up/down
            if (upPressed and downPressed){
                ;
            } else if (upPressed){
                scene.getCamera4D().moveRelativeDefaultU();
            } else if (downPressed){
                scene.getCamera4D().moveRelativeDefaultD();
            }

            // Out/in
            if (outPressed and inPressed){
                ;
            } else if (outPressed){
                scene.getCamera4D().moveRelativeDefaultO();
            } else if (inPressed){
                scene.getCamera4D().moveRelativeDefaultI();
            }

        } else if (scene.getCamera4D().getMovementMode()
                == Camera::MovementMode::Fly){
            /// Move 4d in fly mode

            // Forward/back
            if (forwardPressed and backPressed){
                scene.getCamera4D().applyBrakeFB();
            } else if (forwardPressed){
                scene.getCamera4D().setAccelerationDefaultF();
            } else if (backPressed){
                scene.getCamera4D().setAccelerationDefaultB();
            } else {
                scene.getCamera4D().setAccelerationFB(0);
            }

            // Right/left
            if (rightPressed and leftPressed){
                scene.getCamera4D().applyBrakeRL();
            } else if (rightPressed){
                scene.getCamera4D().setAccelerationDefaultR();
            } else if (leftPressed){
                scene.getCamera4D().setAccelerationDefaultL();
            } else {
                scene.getCamera3D().setAccelerationRL(0);
            }

            // Up/down
            if (upPressed and downPressed){
                scene.getCamera4D().applyBrakeUD();
            } else if (upPressed){
                scene.getCamera4D().setAccelerationDefaultU();
            } else if (downPressed){
                scene.getCamera4D().setAccelerationDefaultD();
            } else {
                scene.getCamera4D().setAccelerationUD(0);
            }

            scene.getCamera4D().moveFly();
        }

        /** Rotation **/
        // Right/left
        if (rotRightPressed and rotLeftPressed){
            ;
        } else if (rotRightPressed){
            scene.getCamera4D().rotateRight();
        } else if (rotLeftPressed){
            scene.getCamera4D().rotateLeft();
        }

        // Up/down
        if (rotUpPressed and rotDownPressed){
            ;
        } else if (rotUpPressed){
            scene.getCamera4D().rotateUp();
        } else if (rotDownPressed){
            scene.getCamera4D().rotateDown();
        }

        // Out/in
        if (rotOutPressed and rotInPressed){
            ;
        } else if (rotOutPressed){
            scene.getCamera4D().rotateOut();
        } else if (rotInPressed){
            scene.getCamera4D().rotateIn();
        }
    }

    glutPostRedisplay();
    glutTimerFunc((int)(Scene::SECONDS_PER_TICK * 1000), timer, dummy);
}

/* Main function: GLUT runs as a console application starting at main()  */
int main(int argc, char** argv){
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


    /** GLUT Processes **/
    init();

    // Initialize GLUT
    glutInit(&argc, argv);

    glutInitDisplayMode(GLUT_RGBA);

    glutInitWindowSize((int)windowWidth, (int)windowHeight);

    // Position the window's initial top-left corner
    glutInitWindowPosition( 100, 100);

    /* create the window and store the handle to it */
    window = glutCreateWindow(/* Fun with Drawing! */ "/* title */");

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
