//
// Created by Jackson Hall on 4/27/2020.
//

#include "Camera.h"
#include <cmath>


/** ---------- Static Const Vars ---------- */
const double Camera::DEFAULT_MOVE_DISTANCE = 0.25;
const double Camera::DEFAULT_ROTATION_ANGLE = 2;

// Credit to:
// https://gaming.stackexchange.com/questions/327830/what-is-the-player-
// acceleration-in-minecraft-when-flying
// and
// https://github.com/ddevault/TrueCraft/wiki/Entity-Movement-And-Physics
const double Camera::FB_ACCEL = 5; // blocks/second^2
const double Camera::RL_ACCEL = 4;
const double Camera::UD_ACCEL = 10;
const double Camera::FB_DRAG = 3; // blocks/second^2
const double Camera::RL_DRAG = 3;
const double Camera::UD_DRAG = 5;
const double Camera::FB_BRAKE = 10; // blocks/second^2
const double Camera::RL_BRAKE = 10;
const double Camera::UD_BREAK = 10;
const double Camera::FB_MAX_SPEED = 3; // blocks/second
const double Camera::RL_MAX_SPEED = 3;
const double Camera::UD_MAX_SPEED = 1;

/** ---------- Constructors ---------- */
Camera::Camera(double focalDistance, MovementMode movementMode) :
        focalDistance(focalDistance), movementMode(movementMode){
}

/** ---------- Static Methods ---------- */
/**
 * Return the orthogonal distance behind the Camera's viewing hyperplane to
 * place the focus such that the Camera renders with a field of view of the
 * given degree angle. If fovDegrees is >= 180, print a warning and return 1.
 */
double Camera::getFocalDistanceFromFOV(const double fovDegrees,
        const double screenWidthBlocks){
    // Focal distance is the cosine of half the field of view
    if (fovDegrees >= 180){
        std::cout << "Warning: Invalid input (fovDegrees) in:"
                     "\n\tdouble Camera::getFocalDistanceFromFOV(const double "
                     "fovDegrees, const double screenWidth)"
                     "\n\t(Camera.cpp)" << std::endl;
        return 1;
    }

    // focal distance is to screen width / 2 as cos(fov/2) is to sin(fov/2)
    return screenWidthBlocks / 2
            * cos(rad(fovDegrees/2)) / sin(rad(fovDegrees/2));
}

/** ---------- Getters ---------- */
double Camera::getFocalDistance() const {
    return focalDistance;
}
Camera::MovementMode Camera::getMovementMode(){
    return movementMode;
}

/** ---------- Setters ---------- */
void Camera::setFocalDistance(const double newFocalDistance){
    focalDistance = newFocalDistance;
}
void Camera::toggleMovementMode(){
    if (movementMode == Camera::MovementMode::Fixed){
        movementMode = Camera::MovementMode::Fly;
    } else {
        movementMode = Camera::MovementMode::Fixed;
    }
}
