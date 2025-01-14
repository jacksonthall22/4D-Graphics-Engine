//
// Created by Jackson Hall on 4/27/2020.
//

#include "Camera.h"
#include <cmath>


/** ---------- Static Const Vars ---------- */
const double Camera::DEFAULT_MOVE_DISTANCE = 0.25;
const double Camera::DEFAULT_ROTATION_ANGLE = 2.5;

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
