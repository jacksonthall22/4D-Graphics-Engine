//
// Created by Jackson Hall on 4/27/2020.
//

#include "Camera.h"
#include <cmath>


/** ---------- Static Fields ---------- */
const double Camera::DEFAULT_MOVE_DISTANCE = 0.25;
const double Camera::DEFAULT_ROTATION_ANGLE = 2;
const double Camera::DEFAULT_FOV = 160;

// Credit to:
// https://gaming.stackexchange.com/questions/327830/what-is-the-player-
// acceleration-in-minecraft-when-flying
const double Camera::FORWARD_ACCELERATION = 4; // blocks/second^2
const double Camera::STRAFE_ACCELERATION  = 4;
const double Camera::UP_ACCELERATION      = 30;
const double Camera::FORWARD_DRAG = 2; // blocks/second^2
const double Camera::STRAFE_DRAG  = 2;
const double Camera::UP_DRAG      = 20;
const double Camera::FORWARD_BRAKE = 100; // blocks/second^2
const double Camera::STRAFE_BRAKE  = 100;
const double Camera::UP_BREAK      = 100;
const double Camera::MAX_FORWARD_VELOCITY = 10; // blocks/second
const double Camera::MAX_STRAFE_VELOCITY = 10;
const double Camera::MAX_UP_VELOCITY = 10;

/** ---------- Constructors ---------- */
Camera::Camera(double focalDistance) : focalDistance(focalDistance) {
}

/** ---------- Static Methods ---------- */
/**
 * Return the orthogonal distance between the focus and a Camera's
 * plane/hyperplane to
 * render with a field of view of the given degree angle.
 */
double Camera::getFocalDistanceFromFOV(const double fovDegrees) {
    // Focal distance is the cosine of half the field of view
    if (fovDegrees >= 180){
        std::cout << "Warning: Invalid input (fovDegrees) in:"
                     "\n\tdouble Camera::getFocalDistanceFromFOV(const double "
                     "fovDegrees, const double screenWidth)"
                     "\n\t(Camera.cpp)" << std::endl;
    }

    // TODO Wonky logic here - FOV should depend on size of screen too
    return cos(rad(fovDegrees / 2.0/* * M_PI / 180 */));
}

/** ---------- Getters ---------- */
double Camera::getFocalDistance() const {
    return focalDistance;
}

/** ---------- Setters ---------- */
void Camera::setFocalDistance(const double newFocalDistance) {
    focalDistance = newFocalDistance;
}

